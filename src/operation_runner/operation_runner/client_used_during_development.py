import sys
import json

import rclpy
from rclpy.node import Node

import time

from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts


class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('client_async') 
       
       #publisher to publish changes to topic
        self.pub = self.create_publisher(msg_type=String,
            topic = 'set_state',
            qos_profile = 10)
        
        #Subscription to state topic:
        self.subscription = self.create_subscription(
            String,
            'state',
            self.listener_callback,
            10) 
        self.subscription  # prevent unused variable warning

        self.state = {'AGV1_allowedToMoveLocation': True, 'AGV1_pos': 'Pos1', 'moveAVG1': False, 'AGV1_moveto':-1,}
        
       
    def listener_callback(self, msg):
        self.get_logger().info('Heard: "%s"' % msg.data) #%s is a place holder for a string in a formating operation
        stateData = json.loads(msg.data)

        for item in stateData:
            self.state[item] = stateData[item] #save the current state values in a 'private' local list
        self.get_logger().info("saved state")
        print('----')
        print(self.getState())
        
        self.check()

    def getState(self):
        return {i: self.state[i] for i in self.state}
    
    def check(self):
        st = self.getState()
        sendRequest = False
        
        if st['AGV1_moveto'] == 0: #0 means warehouse
            sendRequest = True #we want to change the state
            self.get_logger().info("want to move to warehouse")
            st['AGV1_pos'] = 'Warehouse'
            
        elif st['AGV1_moveto'] == 1: # means pos1
            self.get_logger().info("want to move to pos1")
            sendRequest = True #we want to change the state
            st['AGV1_pos'] = 'Pos1'
        
        if sendRequest:
            state_json = json.dumps(st)
            self.pub.publish(String(data = state_json))
            self.get_logger().info("request for state changes sent")
            
        
        
        time.sleep(2)   

def main():
    rclpy.init()
    mc = MinimalClientAsync()

    while rclpy.ok(): 
           rclpy.spin(mc)

    mc.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()