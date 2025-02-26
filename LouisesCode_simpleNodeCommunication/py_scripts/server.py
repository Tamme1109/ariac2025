from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class StateService(Node):
    global state 

    def __init__(self):
        super().__init__('srv')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        
        self.state = [1,0]
        #I'm tyring to publish self.state as a topic:
        self.publisher_ = self.create_publisher(String, 'topic', 10)#Declares that the node publishes messages of type String (imported from the std_msgs.msg module)
        #over a topic named topic, and that the “queue size” is 10. Queue size is a required QoS (quality of service) setting that limits the amount of queued messages if a subscriber is not receiving them fast enough.
        timer_period = 0.2  # seconds #execute every timer_period seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) #timer_callback creates a message with the counter value appended, and publishes it to the console with get_logger().info.
        self.i = 0 #counter used in the callback
        
    def add_two_ints_callback(self, request, response):
        self.setState(request.a, request.b)
        self.get_logger().info('Incoming request, change states\n new state: [%d, %d]' % (request.a, request.b))

        return response

    def timer_callback(self):
        msg = String()
        currState = self.getState()
        msg.data = 'Curr state: [%d, %d], msg nr %d' % (currState[0], currState[1], self.i) #%d operator allows you to add numbers within the strings
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data) #%s is a place holder for a string in a formating operation
        self.i += 1
        
    def setState(self, state1, state2):
        self.state[0] = state1
        self.state[1] = state2
        
    def getState(self):
        return [self.state[0], self.state[1]]
    
def main():
    rclpy.init()

    state_service = StateService()

    rclpy.spin(state_service)

    state_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()