from custom_msg_srv.srv import UpdateDoorState
from custom_msg_srv.msg import Doorstate

from std_msgs.msg import String

import rclpy
from rclpy.node import Node



class StateService(Node):
    global state 

    def __init__(self):
        super().__init__('srv')
        self.srv = self.create_service(UpdateDoorState, 'update_door_state', self.update_door_state_callback)
        
        self.state = [0,0,0,0] #empty state
        self.setState(0,1,0, 0) #set initail state to closed locked door that no one currently wants to pass thorugh and no one has passed through yet.
        
        #publish self.state as a topic:
        self.publisher_ = self.create_publisher(Doorstate, 'topic', 10)#Declares that the node publishes messages of type String (imported from the std_msgs.msg module)
        #over a topic named topic, and that the “queue size” is 10. Queue size is a required QoS (quality of service) setting that limits the amount of queued messages if a subscriber is not receiving them fast enough.
        timer_period = 0.5  # seconds #execute every timer_period seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) #timer_callback creates a message with the counter value appended, and publishes it to the console with get_logger().info.
        self.i = 0 #counter used in the callback
        
    def update_door_state_callback(self, request, response):
        self.setState(request.open, request.locked, request.personwanttopass, request.numpeoplewalkedtrhough)
        self.get_logger().info('Incoming request, change states\n new state: [Open: %d, locked: %d, Person wants to walk through: %d, people who has walked through %d]' % (self.state[0], self.state[1], self.state[2], self.state[3]))
        #response = [request.open, request.locked, request.personwanttopass, request.numpeoplewalkedtrhough]
        return response

        

    def timer_callback(self):
        msg = Doorstate()
        currState = self.getState()
        msg.open = currState[0]
        msg.locked = currState[1]
        msg.personwanttopass = currState[2]
        msg.numpeoplewalkedtrhough = currState[3]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "[%d,%d,%d,%d]"' % (msg.open, msg.locked, msg.personwanttopass, msg.numpeoplewalkedtrhough)) #%s is a place holder for a string in a formating operation
        self.i += 1
        
    def setState(self, open, locked, personwanttopass, numpeoplewalkedtrhough):
        self.state[0] = open
        self.state[1] = locked
        self.state[2] = personwanttopass
        self.state[3] = numpeoplewalkedtrhough
        
    def getState(self):
        return [self.state[0], self.state[1], self.state[2], self.state[3]]
    
def main():
    rclpy.init()

    state_service = StateService()

    rclpy.spin(state_service)

    state_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()