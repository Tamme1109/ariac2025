#!/usr/bin/env python3

import rclpy
import threading
import sys
from rclpy.executors import MultiThreadedExecutor
from ariac_kandidat.competition_interface import CompetitionInterface
from ariac_msgs.msg import CompetitionState
from ariac_kandidat.agv_node import AGVController
from ariac_kandidat.floor_robot import FloorRobot


def main(args=None):
    # Filter out ROS 2 launch arguments like --params-file
    filtered_args = [arg for arg in sys.argv if not arg.startswith('--params-file')]
    rclpy.init(args=args)

    # Spawn all necessary nodes
    agv_control = AGVController()
    interface = CompetitionInterface()

    executor = MultiThreadedExecutor()
    executor.add_node(interface)
    executor.add_node(agv_control)

    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()

    interface.start_competition()

    agv_control.lock_agv_tray(1)
    agv_control.move_agv(1,1)

    interface.move_floor_robot_home()
    try:
        while rclpy.ok():
            if interface.get_competition_state() == CompetitionState.ORDER_ANNOUNCEMENTS_DONE:
                break
    except KeyboardInterrupt:
        pass

    interface.end_competition()

    # ✅ Clean shutdown
    rclpy.shutdown()
    spin_thread.join()


if __name__ == '__main__':
    main()
