#!/usr/bin/env python3

import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor
from ariac_kandidat.competition_interface import CompetitionInterface
from ariac_msgs.msg import CompetitionState
from ariac_kandidat.agv_node import AGVController


def main(args=None):
    rclpy.init(args=args)

    agv_control = AGVController()

    executor = MultiThreadedExecutor()
    executor.add_node(agv_control)

    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()


    agv_control.lock_agv_tray(1)
    agv_control.move_agv(1,1)


if __name__ == '__main__':
    main()
