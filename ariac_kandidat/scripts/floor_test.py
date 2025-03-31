#!/usr/bin/env python3

import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from ariac_kandidat.floor_robot import FloorRobotNode


def main(args=None):
    rclpy.init(args=args)

    # Create the floor robot control node
    floor_robot_control = FloorRobotNode()
    executor = MultiThreadedExecutor()
    executor.add_node(floor_robot_control)

    # Start the executor in a separate thread
    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()

    # Give ROS some time to initialize
    rclpy.spin_once(floor_robot_control)

    try:
        # Move the floor robot to the leftmost position (x = -5.0)
        target_pose = PoseStamped()
        target_pose.header.frame_id = "world"
        target_pose.pose.position.x = -5.0  # Move to the leftmost position along the x-axis
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 0.0
        target_pose.pose.orientation.w = 1.0  # Neutral orientation (no rotation)

        # Send the target pose to the floor robot
        floor_robot_control._move_floor_robot_to_pose(target_pose)

    except KeyboardInterrupt:
        floor_robot_control.get_logger().info("Shutting down due to keyboard interrupt.")

    finally:
        # Shutdown and clean up
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
