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
    #filtered_args = [arg for arg in sys.argv if not arg.startswith('--params-file')]
    rclpy.init(args=args)
    interface = CompetitionInterface()
    executor1 = MultiThreadedExecutor()
    executor1.add_node(interface)
    
    spin_thread = threading.Thread(target=executor1.spin)
    spin_thread.start()

    interface.start_competition()

     # Spawn all necessary nodes
    executor2 = MultiThreadedExecutor()
    agv_control = AGVController()
    #floor_robot = FloorRobot()

    executor2.add_node(agv_control)
    #executor2.add_node(floor_robot)
    
    spin_thread2 = threading.Thread(target=executor2.spin)
    spin_thread2.start()
    

    """agv_control.lock_agv_tray(1)
    agv_control.move_agv(1,2)

    floor_robot.move_home()
    """
    try:
        while rclpy.ok():
            """if interface.get_competition_state() == CompetitionState.ORDER_ANNOUNCEMENTS_DONE:
                break"""
    except KeyboardInterrupt:
        pass

    interface.end_competition()

    # ✅ Clean shutdown
    rclpy.shutdown()
    spin_thread.join()


if __name__ == '__main__':
    main()
