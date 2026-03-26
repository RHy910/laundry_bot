import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# ── Staircase config ──────────────────────────────────────────────────────────
# Each entry is the distance (cm) the sensor reads at that simulated position.
# Positions advance every TICK_RATE seconds.
# Values < 10 = stair detected. Values > 10 = clear.
#
# Layout: robot moves left → right through this timeline.
# front sensor is always SEGMENT_SPACING ticks ahead of mid, mid ahead of back.

SEGMENT_SPACING = 5   # ticks between segments (simulates physical separation)
TICK_RATE       = 0.3  # seconds per tick

# Simulated front sensor readings — mid and back follow with offset
FRONT_TIMELINE = [
    300, 300, 300, 300, 300,   # flat ground — all moving forward
    8,                          # STAIR 1: front hits it
    300, 300, 300, 300, 300,   # front cleared stair 1, advancing
    8,                          # STAIR 2: front hits it
    300, 300, 300, 300, 300,   # front cleared stair 2
    8,                          # STAIR 3: front hits it
    300, 300, 300, 300, 300,   # front cleared stair 3
    300, 300, 300, 300, 300,   # flat at top — done
]

class StaircasePublisher(Node):
    def __init__(self):
        super().__init__('staircase_publisher')

        self.segments = ['front', 'mid', 'back']
        self.ultra_pub = {
            seg: self.create_publisher(Float32, f'ultrasonic/{seg}/distance', 10)
            for seg in self.segments
        }

        # each segment reads from the timeline at a different offset
        self.tick = 0
        self.offsets = {
            'front': 0,
            'mid':   SEGMENT_SPACING,
            'back':  SEGMENT_SPACING * 2,
        }

        self.timer = self.create_timer(TICK_RATE, self.publish_tick)
        self.get_logger().info('Staircase Publisher Started')

    def get_reading(self, seg):
        idx = max(0, self.tick - self.offsets[seg])
        if idx >= len(FRONT_TIMELINE):
            return 300.0  # past the end — open space
        return float(FRONT_TIMELINE[idx])

    def publish_tick(self):
        for seg in self.segments:
            msg = Float32()
            msg.data = self.get_reading(seg)
            self.ultra_pub[seg].publish(msg)
            self.get_logger().info(f'[{seg}] dist: {msg.data}')

        self.tick += 1

        if self.tick > len(FRONT_TIMELINE) + SEGMENT_SPACING * 2 + 5:
            self.get_logger().info('Staircase complete — shutting down simulator')
            raise SystemExit


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