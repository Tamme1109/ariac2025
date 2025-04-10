from functools import partial
from typing import Any, Optional, Tuple, List
import json
from py_ctrl.predicates.state import State
#from py_ctrl.model.model import from_goal_to_goal, the_model, Model
from model_operations import from_goal_to_goal, theModel, Model
from py_ctrl.model.operation import Operation
from py_ctrl.predicates.state import State
from py_ctrl.predicates.errors import NotInStateException

from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import String

import json #for using the json files
import os

class StateService(Node):

    def __init__(self):
        super().__init__('srv')
        
        #initializing states an operations
        self.model: Model = theModel() #the model taht contains the state and the operations
        self.state: State = self.model.initial_state #start with initial state
        self.prev_state = self.state
        
        #Publish self.state as a topic:
        self.publisher_: Publisher = self.create_publisher(
            msg_type = String, topic = 'state', 
            qos_profile = 10,
        )
        
        self.timer = self.create_timer(0.1, self.timer_callback) #timer_callback creates a message with the counter value appended, and publishes it to the console with get_logger().info.
        self.i = 0 #counter used in the callback
        
        #listen for others, others migth want to change the state
        self.create_subscription(
            msg_type = String,
            topic = 'set_state',
            callback = self.set_state_callback,
            qos_profile = 10
        )

    def set_state_callback(self, msg: String):
        #this is where we can do state changes from outside
        try:
            j = msg.data.replace('\'', '\"')
            kvs: dict[str, Any] = json.loads(j)
            print(f"got a state change: {kvs}")
            self.state = self.state.next(**kvs)
        except TypeError:
            pass
        except json.decoder.JSONDecodeError as e:
            print(f"message is bad: {msg.data}")
            print(e)
        
    def timer_callback(self):
        #read from the json file
        f = open(self.state_jason_file_path)
        msg = String()
        msg.data = str(json.load(f))
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data) #%s is a place holder for a string in a formating operation
        self.i += 1
        f.close()
        
        
def main():
    rclpy.init()

    state_service = StateService()

    rclpy.spin(state_service)

    state_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()