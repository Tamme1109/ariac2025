import sys
import json

import rclpy
from rclpy.node import Node

import time

from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async') #
        #client part
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()
        
        #Subscription to topic:
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

       
    def listener_callback(self, msg):
        self.get_logger().info('Heard: "%s"' % msg.data) #%s is a place holder for a string in a formating operation
        stateData = json.loads(msg.data)

        for item in stateData['state']:
            i = 0
            for value in item:
                self.stateValues[i] = value #save the current state values in a 'private' local list
                i += 1
        self.get_logger().info("statevalues %s, %s" %(self.stateValues[0], self.stateValues[1]))
        

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
        future = mc.send_request(wantedState[0], wantedState[1]) #send request to change state
        rclpy.spin_until_future_complete(mc, future)
        
        #response = future.result() #not nececarry rigth now
        mc.get_logger().info(
            'State changed, new: [%d, %d]' %
            (wantedState[0], wantedState[1]))
        
        if mc.stateValues[0] == True:
            mc.get_logger().info('workign when the door is locked.....')
            
        elif mc.stateValues[1]== True:
            mc.get_logger().info('workign really hard over here....')
            
        else:
            mc.get_logger().info('the door is not open and is locked')
           
        time.sleep(2)   
 
    

    mc.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()