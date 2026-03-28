import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time

def create_pose(navigator, x, y, z_orientation, w_orientation):
    """Helper function to create a PoseStamped message."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    # We will set the exact timestamp right before sending the command
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = float(z_orientation)
    pose.pose.orientation.w = float(w_orientation)
    return pose

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Bypass AMCL check since Cartographer is handling localization
    navigator.waitUntilNav2Active(localizer='bt_navigator')

    # Your precise RViz coordinates
    point_a = create_pose(navigator, 13.8186, -11.5037, 0.9997, 0.0217)
    point_b = create_pose(navigator, -11.9592, -15.2132, 0.0336, 0.9994)
    point_c = create_pose(navigator, 2.9836, -14.9824, 0.0166, 0.9998)

    waypoints = [point_a, point_b, point_c]
    loop_count = 1

    print("Initiating Endless Patrol Sequence...")

    try:
        while True:
            print(f"\n--- Starting Patrol Loop #{loop_count} ---")
            
            # CRITICAL: Update the timestamps to 'now' before sending the batch
            # Otherwise Nav2 will reject them as stale data on the second loop
            for wp in waypoints:
                wp.header.stamp = navigator.get_clock().now().to_msg()
                
            # Send the robot through A -> B -> C
            navigator.followWaypoints(waypoints)

            # Monitor the progress of the current loop
            while not navigator.isTaskComplete():
                feedback = navigator.getFeedback()
                if feedback:
                    print(f'Executing current waypoint: {feedback.current_waypoint + 1} of {len(waypoints)}')
                    time.sleep(2.0) # Update the console every 2 seconds

            # Check the result of the loop
            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Loop completed successfully! Resting for 5 seconds before restarting...')
            elif result == TaskResult.CANCELED:
                print('Patrol was canceled. Exiting loop.')
                break
            elif result == TaskResult.FAILED:
                print('Patrol failed (Likely blocked path). Waiting 10 seconds for the area to clear...')
                time.sleep(5.0) 
            
            # Wait 5 seconds before starting the next loop
            time.sleep(5.0)
            loop_count += 1
            
    except KeyboardInterrupt:
        print("\nPatrol manually interrupted by user. Shutting down...")
        
    # Safely cancel any active movement before closing the script
    navigator.cancelTask()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
