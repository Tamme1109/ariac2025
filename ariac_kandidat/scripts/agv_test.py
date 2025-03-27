#!/usr/bin/env python3

import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor
from ariac_kandidat.competition_interface import CompetitionInterface
from ariac_msgs.msg import CompetitionState
from ariac_kandidat.agv_node import AGVController


def main(args=None):
    rclpy.init(args=args)

    # Spawn all neccessary nodes, in this case only the agv for testing
    agv_control = AGVController()
    interface = CompetitionInterface()


    executor = MultiThreadedExecutor()
    executor.add_node(interface)
    executor.add_node(agv_control)

    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()

    interface.start_competition()

    agv_control.lock_agv_tray(1)
    agv_control.move_agv(1)
    

    while rclpy.ok():
        try:
            if interface.get_competition_state() == CompetitionState.ORDER_ANNOUNCEMENTS_DONE:
                break
        except KeyboardInterrupt:
            break

    interface.end_competition()
    
    spin_thread.join()

if __name__ == '__main__':
    main()
