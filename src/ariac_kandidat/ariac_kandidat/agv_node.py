import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from ariac_msgs.srv import MoveAGV
from std_srvs.srv import Trigger
import time
import json
from std_msgs.msg import String

"""import sys
sys.path.append( 'ariac_ws/src/action_interfaces/action')
import AGVAction"""

from actions_operation_runner.action import AGVAction

class AGVController(Node):
    def __init__(self):
        super().__init__('agv_controller')     
        """#publisher to publish changes to topic
        self.pub = self.create_publisher(msg_type=String,
            topic = 'set_state',
            qos_profile = 10)"""
        
        #Subscription to state topic:
        self.subscription = self.create_subscription(
            String,
            'state',
            self.listener_callback,
            10) 
        self.agv_service = ActionServer(self, AGVAction, 'agvaction', self.agv_callback)
        
        self.state = {'AGV1_allowedToMoveLocation': True, 'AGV1_pos': 'Pos1', 'moveAVG1': False, 'AGV1_moveto':-1,} #TODO change later (make it like read the inital state or smt)

    def agv_callback(self, goal_handle): 
        print('agv_action-callback')
        """Callback for agv_action which is requested from the operation runner node"""
        if goal_handle.request.agv_nr == 1:
            print( goal_handle.request.pos)
            if goal_handle.request.pos == 0:
                self.move_agv(1,1)
                print('1 to 1')
            elif goal_handle.request.pos == 1:#currently we only have two options of where we want the agv to move
                self.move_agv(1,0)
                print('1 to 0')
            else:
                print('DOESN\'T WORK :(((((((')
        
        self.get_logger().info('agv has been moved (hopefully)')
        #we should handle som error exeptions here probably (later)
        goal_handle.succeed()
        result = AGVAction.Result()
        result.ok = [goal_handle.request.agv_nr, goal_handle.request.pos]
        print(f'return result:{result}')
        return result
    
    def lock_agv_tray(self, agv_id: int):
        """Locks the tray of the specified AGV (1 or 2)."""
        lock_agv_tray_client = self.create_client(Trigger, f'/ariac/agv{agv_id}_lock_tray')

        self.get_logger().info(f'Waiting for /ariac/agv{agv_id}_lock_tray service...')
        lock_agv_tray_client.wait_for_service()

        request = Trigger.Request()

        future = lock_agv_tray_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(f" Tray on AGV {agv_id} locked.")
        else:
            self.get_logger().error(f" Failed to lock tray on AGV {agv_id}.")

    async def move_agv(self,agv_id: int, destination_id: int):
        """Moves the AGV to a predefined destination by ID (e.g., 1 = station_1)."""
        move_agv_client = self.create_client(MoveAGV, f'/ariac/move_agv{agv_id}')

        self.get_logger().info(f'Waiting for /ariac/move_agv{agv_id} service...')
        move_agv_client.wait_for_service()

        request = MoveAGV.Request()
        request.location = destination_id

        future = move_agv_client.call_async(request)
        #rclpy.spin_until_future_complete(self, future)
        
        res = await future
        print('wait done')

        if res.result().success:
            self.get_logger().info(f" AGV moved to location ID {destination_id}.")
        else:
            self.get_logger().error(" Failed to move AGV.")

    def unlock_agv_tray(self, agv_id: int):
        """Locks the tray of the specified AGV (1 or 2)."""
        unlock_agv_tray_client = self.create_client(Trigger, f'/ariac/agv{agv_id}_lock_tray')

        self.get_logger().info(f'Waiting for /ariac/agv{agv_id}_unlock_tray service...')
        unlock_agv_tray_client.wait_for_service()

        request = Trigger.Request()

        future = unlock_agv_tray_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(f" Tray on AGV {agv_id} unlocked.")
        else:
            self.get_logger().error(f" Failed to unlock tray on AGV {agv_id}.")


    
    def listener_callback(self, msg):
        #self.get_logger().info('Heard: "%s"' % msg.data) #%s is a place holder for a string in a formating operation
        stateData = json.loads(msg.data)

        for item in stateData:
            self.state[item] = stateData[item] #save the current state values in a 'private' local list
        #self.get_logger().info("saved state")
        # print('----')
        #print(self.getState())
        

    def getState(self):
        return {i: self.state[i] for i in self.state}
    
    """Remove these functions later - they are note needed anymore"""
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
    agv_con = AGVController()

    rclpy.spin(agv_con)

    agv_con.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
