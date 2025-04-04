import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import re


from ariac_msgs.msg import (
    AdvancedLogicalCameraImage as AdvancedLogicalCameraImageMsg,
    BreakBeamStatus as BreakBeamStatusMsg,
)

class BreakbeamListener(Node):
    def __init__(self):
        super().__init__('breakbeam_listener')
        # Subscriber to the breakbeam status topic
        self._breakbeam_sub = self.create_subscription(
            BreakBeamStatusMsg,
            '/ariac/sensors/my_breakbeam/status',
            self._breakbeam_cb,
            qos_profile_sensor_data
        )
        
        self.canpPrint = True
        
    def _breakbeam_cb(self, msg):
        data = str(msg)
        #self.get_logger().info(f'detected something: {data[-5:-1]}')
        if data[-5:-1] == 'True' and self.canpPrint:
            self.get_logger().info('AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAa')
            self.canpPrint = False
        elif data[-6:-1] == 'False':
            self.canpPrint = True
    
        
def main(args=None):
    rclpy.init(args=args)
    breakbeamListener = BreakbeamListener()
    rclpy.spin(breakbeamListener)
    breakbeamListener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
