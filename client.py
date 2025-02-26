import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

import time

from std_msgs.msg import String

class MinimalClientAsync(Node):

    global state
    
    def __init__(self):
        super().__init__('minimal_client_async') #
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()
        
        self.state = [0,0] #start default - figure this out better later
        #I'm trying to make it subscribe to a topic here
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10) #The subscriber’s constructor and callback don’t include any timer definition,
                #because it doesn’t need one. Its callback gets called as soon as it receives a message.
        self.subscription  # prevent unused variable warning

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        return self.cli.call_async(self.req)
    
    def stateUpdate(self, a, b):
        self.state[0] = a
        self.state[1] = b
       
    def listener_callback(self, msg):
        self.get_logger().info('Heard: "%s"' % msg.data) #%s is a place holder for a string in a formating operation
        values = ''.join(char for char in msg.data if char.isdigit())
        self.stateUpdate(int(values[0]), int(values[1]))


    def getState(self):
        return [self.state[0], self.state[1]]

def main():
    rclpy.init()

    mc = MinimalClientAsync()
    inp1 = int(sys.argv[1])
    inp2 = int(sys.argv[2])
    
    wantedState = [inp1, inp2]
    i = 0 #if the state is the same for a while, we will change our own desired state
    while True:
        if mc.getState() is not wantedState: #update the state if it's not what you want OBS! DATARACE NOT GOOD LATER (but now i dont care)
            future = mc.send_request(wantedState[0], wantedState[1]) #sebd request to change state
            rclpy.spin_until_future_complete(mc, future)
            
            #response = future.result() #not nececarry rigth now
            mc.get_logger().info(
                'State changed, new: [%d, %d]' %
                (wantedState[0], wantedState[1]))
            
            #mc.stateUpdate(wantedState[0], wantedState[1])
            i = 0 
        elif i ==3:
            wantedState[0] +=1
            wantedState[1] -=1
        
        i += 1
        
        """current = mc.getState()
        mc.get_logger().info(
                'Current state : [%d, %d], i = %d' %
                (current[0], current[1], i))"""
        

        time.sleep(1.5)   
 
    

    mc.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()