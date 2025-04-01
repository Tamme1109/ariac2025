#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ariac_msgs.msg import Order as OrderMsg
from ariac_kandidat.orders import Order, KittingTask

class KittingNode(Node):
    kitting_list: List[KittingTask]
    def __init__(self):
        super().__init__('kitting_node')

        self.subscription = self.create_subscription(
            OrderMsg,  # message type (Order from ariac_msgs)
            '/ariac/orders',  # topic
            self.orders_heard,  # Callback function when a new message is received
            10  # 10 is the queue size
        )

    def _orders_cb(self, msg: OrderMsg):
        '''Callback for the topic /ariac/orders
        Arguments:
            msg -- Order message
        '''
        order = Order(msg)
        if order.order_task == 0:
            self.kitting_list.append(order)


