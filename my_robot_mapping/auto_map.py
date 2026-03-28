import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import numpy as np
import time
import threading
import os

class MapSubscriber(Node):
    """A background node to constantly listen to Cartographer's live map."""
    def __init__(self):
        super().__init__('map_subscriber')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        self.map_msg = None

    def map_callback(self, msg):
        self.map_msg = msg

def create_pose(navigator, x, y):
    """Helper to generate a Goal Pose."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.w = 1.0 
    return pose

def get_frontier_target(map_msg):
    """Analyzes the grid to find the edge of the unknown."""
    width = map_msg.info.width
    height = map_msg.info.height
    resolution = map_msg.info.resolution
    origin_x = map_msg.info.origin.position.x
    origin_y = map_msg.info.origin.position.y

    data = np.array(map_msg.data).reshape((height, width))

    free_space = (data == 0)
    unknown_space = (data == -1)

    up = np.roll(unknown_space, 1, axis=0)
    down = np.roll(unknown_space, -1, axis=0)
    left = np.roll(unknown_space, 1, axis=1)
    right = np.roll(unknown_space, -1, axis=1)

    frontiers = free_space & (up | down | left | right)
    y_indices, x_indices = np.where(frontiers)

    if len(y_indices) < 30:
        return None

    idx = np.random.choice(len(x_indices))
    target_x = (x_indices[idx] * resolution) + origin_x
    target_y = (y_indices[idx] * resolution) + origin_y

    return (target_x, target_y)

def main():
    rclpy.init()
    
    map_node = MapSubscriber()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(map_node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    navigator = BasicNavigator()
    navigator.waitUntilNav2Active(localizer='bt_navigator')

    print("\n--- Starting Autonomous Mapping (Frontier Exploration) ---")
    time.sleep(3.0) 

    try:
        while True:
            if map_node.map_msg is None:
                print("Waiting for Cartographer map data...")
                time.sleep(1.0)
                continue

            target_coords = get_frontier_target(map_node.map_msg)

            if target_coords is None:
                print("\n[SUCCESS] No major frontiers remaining. The area is 100% mapped!")
                break

            target_x, target_y = target_coords
            print(f"\nTargeting new frontier at X: {target_x:.2f}, Y: {target_y:.2f}")

            goal_pose = create_pose(navigator, target_x, target_y)
            navigator.goToPose(goal_pose)

            start_time = time.time()
            while not navigator.isTaskComplete():
                if time.time() - start_time > 60.0:
                    print("[!] Timeout (60s) reached. Target is likely unreachable. Canceling...")
                    navigator.cancelTask()
                    break
                time.sleep(0.5)
                
            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print("[✔] Frontier reached successfully. Surveying area...")
            elif result == TaskResult.FAILED:
                print("[!] Path blocked or failed. Abandoning point and calculating new target...")
            elif result == TaskResult.CANCELED:
                print("[-] Task was canceled (Timeout). Moving on to a different frontier...")

            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nExploration interrupted by user.")
    
    # Return Home
    print("\n--- Returning to Base (0.0, 0.0) ---")
    home_pose = create_pose(navigator, 0.0, 0.0)
    navigator.goToPose(home_pose)
    
    while not navigator.isTaskComplete():
        time.sleep(1.0)
        
    print("Robot has returned home.")

    # AUTO-SAVE THE MAP
    print("\n--- Locking and Saving Map Data ---")
    save_path = "/home/saksham3105/cartographer_ws/src/my_robot_mapping/maps/warehouse_map.pbstream"
    
    print("Finishing Cartographer trajectory...")
    os.system("ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory '{trajectory_id: 0}' > /dev/null 2>&1")
    
    print("Writing map state to disk...")
    os.system(f"ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \"{{filename: '{save_path}'}}\" > /dev/null 2>&1")
    
    print(f"\n[MISSION COMPLETE] Map permanently saved to:\n{save_path}")

    navigator.cancelTask()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
