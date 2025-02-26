import sys

import rclpy
from rclpy.node import Node

import time
import random

from std_msgs.msg import String
from custom_msg_srv.msg import Doorstate
from custom_msg_srv.srv import UpdateDoorState


class MinimalClientAsync(Node):

    global state
    
    def __init__(self):
        super().__init__('minimal_client_async') #
        self.cli = self.create_client(UpdateDoorState, 'update_door_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = UpdateDoorState.Request()       
        self.state = [0,0,0,0] #start default - figure this out better later
        #I'm trying to make it subscribe to a topic here
        self.subscription = self.create_subscription(
            Doorstate,
            'topic',
            self.listener_callback,
            10) #The subscriber’s constructor and callback don’t include any timer definition,
                #because it doesn’t need one. Its callback gets called as soon as it receives a message.
        self.subscription  # prevent unused variable warning

    def send_request(self, open, locked, personwanttopass, numpeoplewalkedtrhough):
        self.req.open = open
        self.req.locked = locked
        self.req.personwanttopass = personwanttopass
        self.req.numpeoplewalkedtrhough = numpeoplewalkedtrhough
        self.get_logger().info('self.req before sending request: %d, %d, %d, %d'%(self.req.open, self.req.locked, self.req.personwanttopass, self.req.numpeoplewalkedtrhough))
        return self.cli.call_async(self.req)
    
    def stateUpdate(self, open, locked, personwanttopass, numpeoplewalkedtrhough):
        self.state[0] = open
        self.state[1] = locked
        self.state[2] = personwanttopass
        self.state[3] = numpeoplewalkedtrhough
       
    def listener_callback(self, msg):
        self.get_logger().info('Heard: "[%d,%d,%d,%d]"' % (msg.open, msg.locked, msg.personwanttopass, msg.numpeoplewalkedtrhough)) #%s is a place holder for a string in a formating operation
        self.stateUpdate(msg.open, msg.locked, msg.personwanttopass, msg.numpeoplewalkedtrhough)


    def getState(self):
        return [self.state[0], self.state[1], self.state[2], self.state[3]]

def main():
    rclpy.init()

    mc = MinimalClientAsync()
    wantedState = [0,0,0,0]
    #randomize a wanted state:
    if random.random() > 0.5:
        if random.random() > 0.5:
            wantedState[0] = 1
        else: wantedState[0] = 0
        if random.random() > 0.75:
            wantedState[1] = 1
        else: wantedState[1] = 0
        if random.random() > 0.4:
            wantedState[2] = 1
        else: wantedState[2] = 0
        
    while True:
        
        if mc.getState() is not wantedState: #update the state if it's not what you want OBS! DATARACE NOT GOOD LATER (but now i dont care)
            future = mc.send_request(wantedState[0], wantedState[1], wantedState[2], wantedState[3]) #sebd request to change state
            rclpy.spin_until_future_complete(mc, future)
            
            #response = future.result() #not nececarry rigth now
            mc.get_logger().info(
                'State changed, new: [%d, %d,%d,%d]' %
                (wantedState[0], wantedState[1], wantedState[2], wantedState[3]))
            
            #mc.stateUpdate(wantedState[0], wantedState[1])


        """current = mc.getState()
        mc.get_logger().info(
                'Current state : [%d, %d], i = %d' %
                (current[0], current[1], i))"""
        

        time.sleep(1.5)   
 
    

    mc.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()