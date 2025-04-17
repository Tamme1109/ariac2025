from functools import partial
from typing import Any, Optional, Tuple, List
import json
from predicates.state import State
from model.model_operations import from_goal_to_goal, theModel, Model
from model.operation import Operation
from predicates.state import State
from predicates.errors import NotInStateException
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.publisher import Publisher
from std_srvs.srv import Trigger
import random


import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool

from actions_operation_runner.action import AGVAction
# ---------------------------------------------------------------------------
# ...
# ---------------------------------------------------------------------------

# publish a goal:

class Runner(Node):
    
    def __init__(self):
        super().__init__('op_runner')  # type: ignore
        self.model: Model = theModel()
        self.state: State = self.model.initial_state
        self.prev_state = self.state
        #idk what I'm doing :(
        self.goal = None
        self.goal_handle = None
        self._send_goal_future = None
        self._get_result_future = None
        self.sendAGVreq = False
        self.req = None
        ## We will not use the goal topic. Should be defind using the state
        self.create_subscription(
            msg_type = String,
            topic = 'set_state',
            callback = self.set_state_callback,
            qos_profile = 10)
        
        #publish state topic
        self.pub_state = self.create_publisher(String, 'state', 10)
        
        #AGV action client
        self.agv_action_client = ActionClient(self, AGVAction, 'agvaction')

        
        #self.get_logger().info(self.model.operations)
        self.timer = self.create_timer(0.1, self.ticker)  # type: ignore #TODO pick an appropriate time for the ticker later

        

    def set_state_callback(self, msg: String):
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
            
        
    def upd_state(self, key: str, value):
        self.state = self.state.next(**{key: value})

    def ticker(self): 
        if self.prev_state != self.state:
            print(f"")
            for k, v in self.state.items():
                print(f"{k}: {v}")
            print(f"")
    
        self.prev_state = self.state

        #call the operation runner
        self.state = simple_operation_runner(self, self.state, self.model)

        
        #publish the state
        state_json = json.dumps(self.state.state)
        self.pub_state.publish(String(data = state_json))   
        
        if self.sendAGVreq == True:
            #send an action goal
            self.req = self.sendRequest() 
            self.sendAGVreq = False 
         

    def sendRequest(self):
        run: bool = self.state.get('AGV1_allowedToMoveLocation')
        if not run and self.state.get('moveAGV1') != 'exec':
            self.upd_state('moveAGV1', 'exec')
            self.goal = AGVAction.Goal()
            self.goal.agv_nr = 1 #we currently only work with one of the agvs
            self.goal.pos = self.state.get('AGV1_moveto')
            
            print("waiting action")
            if self.agv_action_client.wait_for_server(2):
                print("done waiting action")
                self._send_goal_future = self.agv_action_client.send_goal_async(self.goal) # u can add feedback callback here if you want
                self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            #add stuff for failure handling here
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        try:
            result = future.result().result #[agvnr, pos]
            self.get_logger().info('Result: {0}'.format(result.ok))
            print(result.ok[0], result.ok[1])
            self.upd_state(f'moveAGV{result.ok[0]}', 'init')
            self.upd_state('AGV1_pos', result.ok[1])
            #right now we want the AGV to move between the two positions, meaning that if we're at 1 we want to go to 0 etc
            if self.state.get('AGV1_moveto') == 1:
                self.upd_state('AGV1_moveto', 0) 
                
            self.req = None
            self._send_goal_future = None
            self._get_result_future = None
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        
def simple_operation_runner(self, state: State, model: Model) -> State:
        newStateAdded = False
        for operation in model.operations : #check every operation (we want to start every operation we can). Currently we will only start one operation at a time, meaning that when one is found we will break out of the loop - this will ba changed later!
            op: Operation = model.operations[operation]
            op_st: str = state.get(operation)
            if op.eval(state) and op_st == 'i':#evaluate the guard in the precondition of the operation
                next_state = op.start(state)#if we can pass the guard we start the operation and update the state with the new changes
                newStateAdded = True
                self.sendAGVreq = True
                break
            
        for ops in model.operations:#check every running operation and see if they can complete
            op: Operation = model.operations[ops]
            if op.is_completed(state):#if the postcon guard is true, we can finish the operation TODO this doesn't happen right now
                next_state = op.complete(state)
                newStateAdded = True
                print(f"finished {op}")
        
        if not newStateAdded:#if no new operations have started or no eunning operations has finished we keep the state as it is
            next_state = state
            
        return next_state

def main(args=None):
    rclpy.init(args=args)
    
    runner = Runner()
    
    rclpy.spin(runner)
    
    runner.destroy_node()
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()
