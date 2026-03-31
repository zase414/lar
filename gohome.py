from enum import IntEnum, auto
import math

class Stage(IntEnum):
    SEARCHING  = auto()
    ALIGNING   = auto()   # rotate in place
    DRIVING    = auto()   # drive forward
    VERIFYING  = auto()   # check depth equality
    ENTERING   = auto()   # drive through gate

STOP_DIST      = 0.8    # metres - desired distance in front of gate
DEPTH_THRESH   = 0.12   # metres - acceptable depth imbalance
CAMERA_HFOV    = 1.02   # radians - horizontal FOV of your camera (~58°)
IMAGE_WIDTH    = 640
ALIGN_SPEED    = 0.25   # rad/s during in-place rotation
DRIVE_SPEED    = 0.35   # m/s driving toward gate
ENTER_SPEED    = 0.30

class Ferenc:
    def __init__(self):
        self.turtle = Turtlebot(rgb=True)
        self.stop   = False
        self.stage  = Stage.SEARCHING
        # alignment manoeuvre state
        self._target_angle   = 0.0
        self._target_dist    = 0.0
        self._manoeuvre_done = False

    # ------------------------------------------------------------------ #
    def main(self):
        turtle = self.turtle
        sleep(2)
        turtle.register_bumper_event_cb(lambda m: callback_bumper_stop(self, m))
        turtle.register_button_event_cb(lambda m: callback_button0_resume(self, m))
        rate = Rate(10)

        while not turtle.is_shutting_down():
            if self.stop:
                turtle.cmd_velocity(linear=0, angular=0)
                rate.sleep()
                continue

            pylons = self.detect_rectangles(turtle)

            if self.stage == Stage.SEARCHING:
                self._do_searching(turtle, pylons)

            elif self.stage == Stage.ALIGNING:
                self._do_aligning(turtle)

            elif self.stage == Stage.DRIVING:
                self._do_driving(turtle)

            elif self.stage == Stage.VERIFYING:
                self._do_verifying(turtle, pylons)

            elif self.stage == Stage.ENTERING:
                turtle.cmd_velocity(linear=ENTER_SPEED, angular=0)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            rate.sleep()

        cv2.destroyAllWindows()

    # ------------------------------------------------------------------ #
    def _do_searching(self, turtle, pylons):
        """Spin slowly until we see both pylons, then plan the manoeuvre."""
        if pylons is None:
            turtle.cmd_velocity(linear=0, angular=0.2)
            return

        left, right, center = pylons
        depth_L = left[2]
        depth_R = right[2]

        if depth_L is None or depth_R is None:
            turtle.cmd_velocity(linear=0, angular=0.2)
            return

        avg_depth = (depth_L + depth_R) / 2.0

        # ── angle to gate centre ──────────────────────────────────────
        # pixel error of the gate midpoint vs image centre
        pixel_err = (IMAGE_WIDTH / 2) - center[0]          # +ve = gate is to the right
        fov_per_px = CAMERA_HFOV / IMAGE_WIDTH
        angle_to_centre = pixel_err * fov_per_px            # radians to rotate

        # ── lateral offset from depth asymmetry ──────────────────────
        # depth_R > depth_L  →  robot is to the LEFT of centre-line
        depth_diff = depth_R - depth_L                      # metres
        # lateral offset ≈ depth_diff / 2  (small-angle geometry)
        lateral_offset = depth_diff / 2.0                   # metres

        # angle needed to face gate perpendicularly
        # atan2(lateral_offset, avg_depth) gives the extra rotation beyond pointing at centre
        perp_correction = math.atan2(lateral_offset, avg_depth)

        self._target_angle = angle_to_centre + perp_correction
        self._target_dist  = max(0.0, avg_depth - STOP_DIST)
        self._manoeuvre_done = False

        print(f"[PLAN] rotate {math.degrees(self._target_angle):.1f}°, "
              f"drive {self._target_dist:.2f} m  "
              f"(depth_L={depth_L:.2f} depth_R={depth_R:.2f})")

        self.stage = Stage.ALIGNING

    # ------------------------------------------------------------------ #
    def _do_aligning(self, turtle):
        """Rotate in place by _target_angle, using a simple P-controller on yaw."""
        # Here we use a timed open-loop approach (simplest that works).
        # Replace with odometry/IMU integration if available.
        if not hasattr(self, '_align_start'):
            self._align_start     = get_time()
            sign                  = 1 if self._target_angle >= 0 else -1
            # time = angle / speed
            self._align_duration  = abs(self._target_angle) / ALIGN_SPEED
            self._align_direction = sign

        elapsed = get_time() - self._align_start
        if elapsed < self._align_duration:
            turtle.cmd_velocity(linear=0,
                                angular=self._align_direction * ALIGN_SPEED)
        else:
            turtle.cmd_velocity(linear=0, angular=0)
            del self._align_start
            self._drive_start    = get_time()
            self._drive_duration = self._target_dist / DRIVE_SPEED
            self.stage = Stage.DRIVING

    # ------------------------------------------------------------------ #
    def _do_driving(self, turtle):
        """Drive straight for the planned distance."""
        elapsed = get_time() - self._drive_start
        if elapsed < self._drive_duration:
            turtle.cmd_velocity(linear=DRIVE_SPEED, angular=0)
        else:
            turtle.cmd_velocity(linear=0, angular=0)
            self.stage = Stage.VERIFYING

    # ------------------------------------------------------------------ #
    def _do_verifying(self, turtle, pylons):
        """Check if pylons are equidistant; enter or retry."""
        if pylons is None:
            # Can't see pylons - nudge forward a little and retry
            turtle.cmd_velocity(linear=0.1, angular=0)
            return

        left, right, _ = pylons
        depth_L, depth_R = left[2], right[2]

        if depth_L is None or depth_R is None:
            turtle.cmd_velocity(linear=0, angular=0)
            return

        imbalance = abs(depth_R - depth_L)
        print(f"[VERIFY] imbalance={imbalance:.3f} m  (threshold={DEPTH_THRESH})")

        if imbalance < DEPTH_THRESH:
            print("[VERIFY] ✓ aligned — entering gate")
            self.stage = Stage.ENTERING
        else:
            print("[VERIFY] ✗ misaligned — retrying from scratch")
            self.stage = Stage.SEARCHING