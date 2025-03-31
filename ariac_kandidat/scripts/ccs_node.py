#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ariac_msgs.msg import Order

class CCSNode(Node):
    def __init__(self):
        super().__init__('ccs_node')

        self.subscription = self.create_subscription(
            Order,  # message type (Order from ariac_msgs)
            '/ariac/orders',  # topic
            self.orders_heard,  # Callback function when a new message is received
            10  # 10 is the queue size
        )

    def orders_heard(self, msg):
        self.get_logger().info(f"Received Order ID: {msg.id}")


def main(args=None):
    rclpy.init(args=args)
    node = CCSNode()


    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()