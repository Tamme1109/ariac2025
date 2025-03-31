#!/usr/bin/env python3

import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor
from ariac_kandidat.competition_interface import CompetitionInterface
from ariac_msgs.msg import CompetitionState

def main(args=None):
    rclpy.init(args=args)
    interface = CompetitionInterface()
    executor = MultiThreadedExecutor()
    executor.add_node(interface)

    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()

    interface.start_competition()

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