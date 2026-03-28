#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def main():
    rclpy.init()
    
    # Start the Nav2 Commander
    navigator = BasicNavigator()

    # THE FIX: Tell Nav2 not to wait for AMCL!
    navigator.waitUntilNav2Active(localizer='bt_navigator')

    # 1. Define your Semantic Zones
    warehouse_zones = {
        'production': {'x': 2.5, 'y': 1.0, 'w': 1.0},
        'storage':    {'x': -3.0, 'y': 4.5, 'w': 1.0},
        'packaging':  {'x': 6.0, 'y': -2.0, 'w': 1.0}
    }

    # Ask the user where to send the robot
    print("\n--- Warehouse Dispatch System ---")
    print("Available Zones: production, storage, packaging")
    target_zone = input("Where should the robot go? ").lower()

    if target_zone not in warehouse_zones:
        print("Error: Unknown zone. Shutting down.")
        rclpy.shutdown()
        return

    # 2. Package the coordinates into a ROS 2 Pose message
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = warehouse_zones[target_zone]['x']
    goal_pose.pose.position.y = warehouse_zones[target_zone]['y']
    goal_pose.pose.orientation.w = warehouse_zones[target_zone]['w']

    # 3. Send the command to the robot!
    print(f"Routing robot to {target_zone.upper()} area...")
    navigator.goToPose(goal_pose)

    # 4. Monitor the progress
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f"Distance remaining: {feedback.distance_remaining:.2f} meters", end='\r')

    # 5. Final Result
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print(f"\nSuccess! Robot has arrived at the {target_zone.upper()} area.")
    elif result == TaskResult.CANCELED:
        print("\nTask was canceled.")
    elif result == TaskResult.FAILED:
        print("\nRobot failed to reach the destination. Is the path blocked?")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
