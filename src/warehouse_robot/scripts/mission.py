#!/usr/bin/env python3
import time
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

def main():
    rclpy.init()

    # 1. Start the Navigator
    navigator = BasicNavigator()

    # 2. Set Initial Pose (Simulating the "2D Pose Estimate" click)
    # We assume the robot starts at (0,0) facing East (0.0 orientation)
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    
    print("Setting Initial Pose...")
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2 to activate
    navigator.waitUntilNav2Active()

    # 3. Define the Goal (Go to Shelf A)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    
    # --- UPDATE THESE COORDINATES FOR YOUR MAP ---
    goal_pose.pose.position.x = 2.0  # Change X
    goal_pose.pose.position.y = 0.5  # Change Y
    goal_pose.pose.orientation.w = 1.0
    # ---------------------------------------------

    print(f"Going to goal: {goal_pose.pose.position.x}, {goal_pose.pose.position.y}")
    navigator.goToPose(goal_pose)

    # 4. Monitor Progress
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f'Distance remaining: {feedback.distance_remaining:.2f} meters', end='\r')
            
    # 5. Check Result
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("\nGoal Reached! Mission Complete.")
    elif result == TaskResult.CANCELED:
        print("\nGoal was canceled!")
    elif result == TaskResult.FAILED:
        print("\nGoal failed!")

    navigator.lifecycleShutdown()
    exit(0)

if __name__ == '__main__':
    main()