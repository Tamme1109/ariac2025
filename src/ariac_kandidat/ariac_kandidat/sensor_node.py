import rclpy
from rclpy.node import Node

from ariac_msgs.msg import (
    BreakbeamStatus as BreakBeamStatusMsg,
    AdvancedLogicalCameraImage as AdvancedLogicalCameraImageMsg
)

class SensorNode(Node):
    def __init__(self):
        print("Bomba")