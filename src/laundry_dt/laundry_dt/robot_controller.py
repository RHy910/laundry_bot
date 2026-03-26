import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
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
        self.steppers = ['front', 'back']  # front stepper on mid segment

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

        for stp in self.steppers:  # fix: was using wrong variable name
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
        self.get_logger().info('Robot Controller Started')

    # ── helpers ───────────────────────────────────────────────────────

    def drive(self, seg, speed=0.5):
        cmd = Twist()
        cmd.linear.x = speed
        self.stopped[seg] = False
        self.motor_pub[seg].publish(cmd)

    def halt(self, seg):
        self.motor_pub[seg].publish(Twist())
        self.stopped[seg] = True
        self.get_logger().info(f'*** [{seg}] stopped ***')

    def stop_all(self):
        for seg in self.segments:
            self.motor_pub[seg].publish(Twist())
            self.stopped[seg] = True
        self.get_logger().info('*** all stopped ***')

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
        self.get_logger().info(f'Winch activated — {self.winch_steps} steps')

    def do_lift(self, seg):
        """Execute a lift for the given segment using stored stair height."""
        if self.level[seg] not in self.stair_heights:
            self.get_logger().error(
                f'[{seg}] cannot lift — no height stored for level {self.level[seg]}'
            )
            return

        steps = self.stair_heights[self.level[seg]]

        if seg == 'front':
            self.step_up('front', steps)
        elif seg == 'mid':
            self.step_down('front', steps)  # front stepper lowers → lifts mid
            self.step_up('back', steps)     # back stepper raises → lifts mid
        else:
            self.step_down('back', steps)   # back stepper lowers back segment

        
        self.get_logger().info(
            f'[{seg}] lifting → level {self.level[seg]} | steps: {steps}'
        )

    def get_active_seg(self):
        """
        Determines which segment should be active based on level state.

        Level patterns and active segment:
        {f:0, m:0, b:0} → front  (f == m, front leads)
        {f:1, m:0, b:0} → mid    (f > m, mid == b, mid catches up)
        {f:1, m:1, b:0} → back   (f == m > b, back catches up)
        {f:1, m:1, b:1} → front  (all equal, front leads again)
        """
        f = self.level.get('front', 0)
        m = self.level.get('mid',   0)
        b = self.level.get('back',  0)

        if not (f >= m >= b):
            self.get_logger().error(
                f'INVALID level order — must be front >= mid >= back | '
                f'levels: {self.level}'
            )
            return None

        if f == m:
            return 'front'   # front and mid equal — front leads next lift
        elif m == b:
            return 'mid'     # front ahead, mid and back equal — mid catches up
        else:
            return 'back'    # front and mid ahead — back catches up

    # ── stepper callback ──────────────────────────────────────────────

    def on_stepper(self, msg, stp):
        self.live_steps[stp] = msg.data

        # front stepper scouts stair height while lifting
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
        distance  = msg.data
        active    = self.get_active_seg()

        if active is None:
            return  # invalid state — sanity guard fired, don't process

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

                if seg == 'front':
                    # front always scouts — lift indefinitely until ultrasonic clears
                    self.scouting_level = self.level['front']+1
                    self.state          = RobotState.LIFTING
                    self.step_up('front', 999.0)
                    self.get_logger().info(f'--- Scouting stair {self.scouting_level} ---')
                else:
                    # mid or back — use stored height
                    self.state = RobotState.LIFTING
                    self.do_lift(seg)

        # ── LIFTING ───────────────────────────────────────────────────
        if self.state == RobotState.LIFTING and seg == active:
            if distance > self.stair_threshold:
                self.get_logger().info(f'[{seg}] cleared — moving forward')

                if seg == 'back':
                    # back cleared — winch pulls it forward
                    self.state       = RobotState.BACK_WINCHING
                    self.winch_count = 0
                    self.fire_winch()
                else:
                    self.state = RobotState.SEGMENT_FORWARD
                    for s in self.segments:
                        self.drive(s)

        # ── SEGMENT FORWARD ───────────────────────────────────────────
        if self.state == RobotState.SEGMENT_FORWARD:
            if distance < self.stair_threshold:
                self.halt(seg)
                self.stopped[seg] = True

            # once the active segment has stopped, update levels and lift next
            # ADD CHECK: if distance is beyond full bot length → adjust levels
            if all(self.stopped.values()):  # fix: was set(stopped) == set(True)
                self.state = RobotState.LIFTING
                for s in self.segments:
                    self.drive(s)
                self.get_logger().info(
                    f'All segments positioned | levels: {self.level} — advancing'
                )
                
                previous_active = active
                self.level[active] += 1  # update level of just-lifted segment
                active = self.get_active_seg()  # re-evaluate active segment based on new levels
                
                self.get_logger().info(
                    f'{active} lifted → next active: {active} | levels: {self.level}'
                )

                self.do_lift(active)  # lift next segment based on current active pattern
                self.state = RobotState.LIFTING  # ensure state is set to LIFTING for next lift

        # ── BACK WINCHING ─────────────────────────────────────────────
        if self.state == RobotState.BACK_WINCHING:
            self.winch_count += 1
            if self.winch_count >= self.winch_steps:
                self.state = RobotState.ADVANCING
                self.stopped['back'] = False
                self.drive('back')
                self.get_logger().info('Winch complete — back advancing')

        # ── STAIR COMPLETE ────────────────────────────────────────────
        if (self.state == RobotState.ADVANCING and
                distance > 200.0 and
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
            f'heights: {self.stair_heights}'
        )


def main():
    rclpy.init()
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()