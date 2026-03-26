import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String
from rclpy.node import Node
from enum import Enum


class RobotState(Enum):
    IDLE             = 0
    ADVANCING        = 1
    LIFTING          = 2
    SEGMENT_FORWARD  = 3
    BACK_WINCHING    = 4
    COMPLETE         = 5


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        self.state    = RobotState.IDLE
        self.segments = ['front', 'mid', 'back']
        self.steppers = ['front', 'back']

        self.level   = {'front': 0, 'mid': 0, 'back': 0}
        self.stopped = {'front': True, 'mid': True, 'back': True}

        self.stair_heights  = {}
        self.scouting_level = 0
        self.live_steps     = {'front': 0, 'back': 0}

        # ── tuning ────────────────────────────────────────────────────
        self.stair_threshold  = 10.0
        self.clearance_offset = 20
        self.winch_steps      = 200
        self.winch_count      = 0
        self.winch_duration   = 3.0
        self.winch_start_time = None

        # ── dead reckoning ────────────────────────────────────────────
        self.drive_speed      = 0.5
        self.segment_length   = 30.0
        self.drive_start_time = {'front': None, 'mid': None, 'back': None}

        # ── top detection ─────────────────────────────────────────────
        self.front_at_top = False

        # ── publishers / subscribers ──────────────────────────────────
        self.motor_pub   = {}
        self.stepper_pub = {}
        self.stepper_sub = {}
        self.ultra_sub   = {}

        for seg in self.segments:
            self.motor_pub[seg] = self.create_publisher(
                Twist, f'{seg}/cmd_vel', 10
            )
            self.ultra_sub[seg] = self.create_subscription(
                Float32,
                f'ultrasonic/{seg}/distance',
                lambda msg, s=seg: self.on_ultrasonic(msg, s),
                10
            )

        for stp in self.steppers:
            self.stepper_pub[stp] = self.create_publisher(
                Float32, f'stepper/{stp}/cmd', 10
            )
            self.stepper_sub[stp] = self.create_subscription(
                Float32,
                f'stepper/{stp}/steps',
                lambda msg, s=stp: self.on_stepper(msg, s),
                10
            )

        self.winch_pub = self.create_publisher(Float32, '/mid/winch/cmd', 10)
        self.lift_pub  = self.create_publisher(String, '/segment_lifted', 10)
        self.get_logger().info('Robot Controller Started')

    # ── helpers ───────────────────────────────────────────────────────

    def drive(self, seg, speed=0.5):
        cmd = Twist()
        cmd.linear.x = speed
        self.stopped[seg] = False
        self.drive_start_time[seg] = self.get_clock().now()
        self.motor_pub[seg].publish(cmd)

    def halt(self, seg):
        self.motor_pub[seg].publish(Twist())
        self.stopped[seg] = True
        self.drive_start_time[seg] = None
        self.get_logger().info(f'*** [{seg}] stopped ***')

    def stop_all(self):
        for seg in self.segments:
            self.motor_pub[seg].publish(Twist())
            self.stopped[seg] = True
            self.drive_start_time[seg] = None
        self.get_logger().info('*** all stopped ***')

    def distance_traveled(self, seg):
        if self.drive_start_time[seg] is None:
            return 0.0
        elapsed = (self.get_clock().now() - self.drive_start_time[seg]).nanoseconds / 1e9
        return elapsed * self.drive_speed * 100.0

    def step_up(self, stp, steps):
        cmd = Float32()
        cmd.data = float(steps)
        self.stepper_pub[stp].publish(cmd)

    def step_down(self, stp, steps):
        cmd = Float32()
        cmd.data = float(-steps)
        self.stepper_pub[stp].publish(cmd)

    def fire_winch(self):
        cmd = Float32()
        cmd.data = float(self.winch_steps)
        self.winch_pub.publish(cmd)
        self.winch_start_time = self.get_clock().now()
        self.get_logger().info(f'Winch activated — {self.winch_steps} steps')

    def do_lift(self, seg):
        """Execute a lift for the given segment using stored stair height."""
        if seg == 'front':
            self.scouting_level = self.level['front'] + 1
            self.step_up('front', 999.0)
            self.get_logger().info(f'[front] scouting stair {self.scouting_level}')
            lift_msg = String()
            lift_msg.data = 'front'
            self.lift_pub.publish(lift_msg)
            return True

        target_level = self.level[seg] + 1
        if target_level not in self.stair_heights:
            self.get_logger().error(
                f'[{seg}] cannot lift — no height stored for level {target_level}'
            )
            return False

        steps = self.stair_heights[target_level]

        if seg == 'mid':
            self.step_down('front', steps)
            self.step_up('back', steps)
        else:
            self.step_down('back', steps)

        self.get_logger().info(
            f'[{seg}] lifting → level {self.level[seg]} | steps: {steps}'
        )
        lift_msg = String()
        lift_msg.data = seg
        self.lift_pub.publish(lift_msg)
        return True

    def get_active_seg(self):
        f = self.level.get('front', 0)
        m = self.level.get('mid',   0)
        b = self.level.get('back',  0)

        if not (f >= m >= b):
            self.get_logger().error(
                f'INVALID level order — must be front >= mid >= back | '
                f'levels: {self.level}'
            )
            return None

        # if front is at top, it can no longer lead — mid catches up
        # if mid has also caught up to front, back catches up
        if self.front_at_top:
            if m < f:
                return 'mid'
            elif b < m:
                return 'back'
            else:
                return None  # all equal, complete

        if f == m:
            return 'front'
        elif m == b:
            return 'mid'
        else:
            return 'back'

    # ── stepper callback ──────────────────────────────────────────────

    def on_stepper(self, msg, stp):
        self.live_steps[stp] = msg.data

        if stp == 'front' and self.state == RobotState.LIFTING:
            if self.scouting_level not in self.stair_heights:
                measured = msg.data + self.clearance_offset
                self.stair_heights[self.scouting_level] = measured
                self.get_logger().info(
                    f'Stair {self.scouting_level} scouted: {measured} steps | '
                    f'heights: {self.stair_heights}'
                )

    # ── ultrasonic callback ───────────────────────────────────────────

    def on_ultrasonic(self, msg, seg):
        distance = msg.data
        active   = self.get_active_seg()

        if active is None:
            if (self.front_at_top and
                    self.level['front'] == self.level['mid'] == self.level['back'] and
                    self.level['front'] > 0 and
                    self.state != RobotState.COMPLETE):
                self.state = RobotState.COMPLETE
                self.stop_all()
                self.get_logger().info(
                    f'STAIR CLIMB COMPLETE — reached level {self.level["front"]}'
                )
            return

        # ── IDLE → ADVANCING ─────────────────────────────────────────
        if self.state == RobotState.IDLE:
            self.state = RobotState.ADVANCING
            for s in self.segments:
                self.drive(s)
            return

        # ── ADVANCING ────────────────────────────────────────────────
        if self.state == RobotState.ADVANCING and seg == active:
            if distance < self.stair_threshold:
                self.stop_all()
                self.get_logger().info(
                    f'[{seg}] hit stair | level: {self.level[seg]}'
                )
                self.state = RobotState.LIFTING
                self.do_lift(seg)

        # ── LIFTING ───────────────────────────────────────────────────
        if self.state == RobotState.LIFTING and seg == active:
            if distance > self.stair_threshold:
                self.get_logger().info(f'[{seg}] cleared — moving forward')
                self.state = RobotState.SEGMENT_FORWARD
                for s in self.segments:
                    self.drive(s)

        # ── SEGMENT FORWARD ───────────────────────────────────────────
        if self.state == RobotState.SEGMENT_FORWARD:
            traveled = self.distance_traveled(seg)
            self.get_logger().info(
                f'[SF] [{seg}] dist: {distance:.1f} | traveled: {traveled:.1f}cm | '
                f'stopped: {self.stopped[seg]} | threshold: {self.stair_threshold} | '
                f'segment_length: {self.segment_length}'
            )

            # stop condition 1: ultrasonic detects next stair
            if distance < self.stair_threshold and not self.stopped[seg]:
                self.get_logger().info(f'[SF] [{seg}] STAIR DETECTED — halting')
                self.halt(seg)

            # front sees landing signal (301) — just flag it, let normal flow stop and credit level
            if seg == 'front' and not self.front_at_top and distance > 300:
                self.front_at_top = True
                self.get_logger().info('[front] landing signal received — front at top')

            # stop condition 2: traveled a full segment length with no stair
            if not self.stopped[seg] and traveled >= self.segment_length:
                self.get_logger().info(
                    f'[SF] [{seg}] DISTANCE CUTOFF at {traveled:.1f}cm — assuming landing, halting'
                )
                self.halt(seg)

            self.get_logger().info(
                f'[SF] stopped state — front: {self.stopped["front"]} | '
                f'mid: {self.stopped["mid"]} | back: {self.stopped["back"]} | '
                f'all stopped: {all(self.stopped.values())}'
            )

            if all(self.stopped.values()):
                self.level[active] += 1
                next_active = self.get_active_seg()

                self.get_logger().info(
                    f'[{active}] positioned | levels: {self.level} '
                    f'→ next active: {next_active}'
                )

                # all levels equal and front confirmed at top — complete
                if (self.front_at_top and
                        self.level['front'] == self.level['mid'] == self.level['back']):
                    self.state = RobotState.COMPLETE
                    self.stop_all()
                    self.get_logger().info(
                        f'STAIR CLIMB COMPLETE — reached level {self.level["front"]}'
                    )
                    return

                if next_active == 'back':
                    self.state       = RobotState.BACK_WINCHING
                    self.winch_count = 0
                    self.fire_winch()
                else:
                    if self.do_lift(next_active):
                        self.state = RobotState.LIFTING
                    else:
                        self.state = RobotState.IDLE
                        self.get_logger().error(
                            f'Lift failed for [{next_active}] — stalling | levels: {self.level}'
                        )

        # ── BACK WINCHING ─────────────────────────────────────────────
        if self.state == RobotState.BACK_WINCHING:
            elapsed = (self.get_clock().now() - self.winch_start_time).nanoseconds / 1e9
            self.get_logger().info(f'[WINCH] elapsed: {elapsed:.2f}s / {self.winch_duration}s')
            if elapsed >= self.winch_duration:
                self.state = RobotState.ADVANCING
                self.drive('back')
                self.get_logger().info('Winch complete — back advancing')

        # ── STAIR COMPLETE (advancing check) ─────────────────────────
        if (self.state == RobotState.ADVANCING and
                self.front_at_top and
                self.level['front'] == self.level['mid'] == self.level['back'] and
                self.level['front'] > 0):
            self.state = RobotState.COMPLETE
            self.stop_all()
            self.get_logger().info(
                f'STAIR CLIMB COMPLETE — reached level {self.level["front"]}'
            )

        self.get_logger().info(
            f'[{seg}] dist: {distance:.1f} | active: {active} | '
            f'levels: {self.level} | state: {self.state.name} | '
            f'front_at_top: {self.front_at_top} | heights: {self.stair_heights}'
        )


def main():
    rclpy.init()
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()