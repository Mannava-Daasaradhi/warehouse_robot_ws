#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

class RandomWalk(Node):
    def __init__(self):
        super().__init__('random_walk')
        
        # Subscribe to Lidar (Best Effort to match Gazebo)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )
        
        # Publisher for Movement
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.move_cmd = Twist()
        self.linear_speed = 0.3  # Forward speed
        self.angular_speed = 1.0 # Turning speed
        self.min_dist = 0.8      # Distance to trigger a turn

        self.get_logger().info("Random Walk Node Started! Monitoring Lidar...")

    def scan_callback(self, msg):
        # The Lidar gives us 360 degrees of data. 
        # We only care about the front 60 degrees (-30 to +30).
        # In this array, index 0 is usually front, or it wraps around.
        # Let's assume standard behavior: front is in the middle or ends.
        
        # Simplified logic: Check the "Front" cone
        # If the array starts at -PI, the middle is index len/2.
        
        ranges = msg.ranges
        mid_index = len(ranges) // 2
        window_width = 40  # Check 20 rays on each side of center
        
        # Get the minimum distance in the front cone
        front_ranges = ranges[mid_index - window_width : mid_index + window_width]
        
        # Filter out invalid 'inf' or 'nan'
        valid_front = [r for r in front_ranges if 0.1 < r < 20.0]
        
        if not valid_front:
            min_front_dist = 10.0 # Clear path if no data
        else:
            min_front_dist = min(valid_front)

        # --- LOGIC ---
        if min_front_dist < self.min_dist:
            # OBSTACLE AHEAD! Turn!
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = self.angular_speed
            self.get_logger().info(f"Obstacle at {min_front_dist:.2f}m! Turning...", once=True)
        else:
            # ALL CLEAR! Drive!
            self.move_cmd.linear.x = self.linear_speed
            self.move_cmd.angular.z = 0.0
        
        self.cmd_pub.publish(self.move_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = RandomWalk()
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.shutdown()

if __name__ == '__main__':
    main()