import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import sys

class LidarValidator(Node):
    def __init__(self):
        super().__init__('lidar_validator')
        
        # CHANGED: Use 'qos_profile_sensor_data' (Best Effort) 
        # to match Gazebo's default publishing behavior.
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile_sensor_data
        )
        
        self.timer = self.create_timer(5.0, self.timeout_callback)
        self.get_logger().info('waiting for lidar data (Best Effort QoS)...')

    def listener_callback(self, msg):
        if not msg.ranges:
            return

        # Filter valid points
        valid_points = [r for r in msg.ranges if 0.0 < r < 100.0]
        
        if len(valid_points) > 0:
            print("\n" + "="*40)
            print("✅ PASS: LIDAR IS WORKING")
            print(f"   - Valid Points: {len(valid_points)}")
            print(f"   - Nearest Obstacle: {min(valid_points):.2f} m")
            print("="*40 + "\n")
            sys.exit(0)

    def timeout_callback(self):
        print("\n" + "="*40)
        print("❌ FAIL: TIMED OUT")
        print("   - No data received.")
        print("   - Verify 'gz topic -l' shows '/scan'")
        print("="*40 + "\n")
        sys.exit(1)

def main(args=None):
    rclpy.init(args=args)
    validator = LidarValidator()
    try:
        rclpy.spin(validator)
    except SystemExit:
        rclpy.shutdown()

if __name__ == '__main__':
    main()