import rclpy
import random
from rclpy.node import Node
from std_msgs.msg import Float32, String

TICK_RATE        = 0.3
START_DIST       = 300
STAIR_THRESHOLD  = 8
DECREMENT        = 20
NUM_STAIRS      = 0
STAIR_HEIGHT_MIN = 500
STAIR_HEIGHT_MAX = 1500


class StaircasePublisher(Node):
    def __init__(self):
        super().__init__('staircase_publisher')

        self.segments = ['front', 'mid', 'back']

        # pointer = which stair this segment is currently approaching
        # only front starts at 1 (approaching stair 1), others wait
        self.pointers = {'front': 1, 'mid': 0, 'back': 0}

        # current reading per segment — decrements each tick toward threshold
        self.readings = {'front': START_DIST, 'mid': START_DIST, 'back': START_DIST}

        # ultrasonic publishers
        self.ultra_pub = {
            seg: self.create_publisher(Float32, f'ultrasonic/{seg}/distance', 10)
            for seg in self.segments
        }

        # stepper feedback — simulates front measuring stair height
        self.stepper_pub = self.create_publisher(Float32, 'stepper/front/steps', 10)

        # lift confirmations from robot controller
        self.lift_sub = self.create_subscription(
            String, '/segment_lifted', self.on_lift, 10
        )

        self.timer = self.create_timer(TICK_RATE, self.publish_tick)
        self.get_logger().info('Staircase Publisher Started')

    def on_lift(self, msg):
        seg = msg.data
        if seg not in self.pointers:
            return

        self.pointers[seg] += 1        # advance to next stair
        self.readings[seg] = START_DIST  # reset reading for next approach

        self.get_logger().info(
            f'[{seg}] lifted — now approaching stair {self.pointers[seg]}'
        )

        # unlock next segment
        if seg == 'front' and self.pointers['mid'] == 0:
            self.pointers['mid'] = 1
            self.get_logger().info('[mid] unlocked — starting approach')
        elif seg == 'mid' and self.pointers['back'] == 0:
            self.pointers['back'] = 1
            self.get_logger().info('[back] unlocked — starting approach')

        # front publishes random stair height for controller to store
        if seg == 'front':
            height = float(random.randint(STAIR_HEIGHT_MIN, STAIR_HEIGHT_MAX))
            step_msg = Float32()
            step_msg.data = height
            self.stepper_pub.publish(step_msg)
            self.get_logger().info(f'[front] stair height: {height} steps')

    def publish_tick(self):
        for seg in self.segments:
            # only decrement if unlocked and not past last stair
            if self.pointers[seg] > 0 and self.pointers[seg] <= NUM_STAIRS+1:
                if self.readings[seg] > STAIR_THRESHOLD:
                    self.readings[seg] -= DECREMENT
                    self.readings[seg] = max(self.readings[seg], STAIR_THRESHOLD)
            elif self.pointers[seg] > NUM_STAIRS+1:
                # past last stair — send 301 as explicit landing signal
                self.readings[seg] = 301

            msg = Float32()
            msg.data = float(self.readings[seg])
            self.ultra_pub[seg].publish(msg)
            self.get_logger().info(
                f'[{seg}] dist: {msg.data} | stair: {self.pointers[seg]}'
            )


def main():
    rclpy.init()
    node = StaircasePublisher()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()