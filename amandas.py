import rclpy
import json
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String

hejhej

hejdå

class StateManager(Node):
    """ 
    Node 2: Manages system state.
    Provides a service to toggle the state and publishes updates.
    """

    def __init__(self):
        super().__init__('state_manager')
        self.state = {"mode": "IDLE", "robot_position": "home", "operations": []}
        self.publisher = self.create_publisher(String, 'state_topic', 10)
        self.service = self.create_service(Trigger, 'change_state', self.handle_state_change)
        self.get_logger().info("State Manager initialized.")

    def handle_state_change(self, request, response):
        """Handles requests to toggle the system state."""
        self.get_logger().info(f"Received request. Current state: {self.state}")
        self.state["mode"] = "RUNNING" if self.state["mode"] == "IDLE" else "IDLE"
        msg = String()
        msg.data = json.dumps(self.state)
        self.publisher.publish(msg)

        response.success = True
        response.message = f"State changed to {self.state['mode']}"
        return response


class ControlNode(Node):
    """ 
    Node 1: Sends requests to change the state and listens for updates.
    """

    def __init__(self):
        super().__init__('control_node')
        self.subscription = self.create_subscription(String, 'state_topic', self.state_callback, 10)
        self.client = self.create_client(Trigger, 'change_state')

        self.get_logger().info("Control Node initialized.")
        self.timer = self.create_timer(5.0, self.request_state_change)

    def state_callback(self, msg):
        """Receives and logs state updates."""
        state_data = json.loads(msg.data)
        self.get_logger().info(f"State Update: {state_data}")

    def request_state_change(self):
        """Requests a state change."""
        if not self.client.service_is_ready():
            self.get_logger().warn("State service unavailable.")
            return

        request = Trigger.Request()
        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        """Handles the response from the state manager."""
        try:
            response = future.result()
            self.get_logger().info(f"State change successful: {response.message}")
        except Exception as e:
            self.get_logger().error(f"State service call failed: {e}")


class MonitoringNode(Node):
    """
    Node 3: Monitors state changes but does not request changes.
    This node is meant for passive monitoring/logging.
    """

    def __init__(self):
        super().__init__('monitoring_node')
        self.subscription = self.create_subscription(String, 'state_topic', self.state_callback, 10)
        self.get_logger().info("Monitoring Node initialized.")

    def state_callback(self, msg):
        """Logs state updates for debugging."""
        state_data = json.loads(msg.data)
        self.get_logger().info(f"MONITOR: Detected state change: {state_data}")


class OperationRunner(Node):
    """
    Node 4: Executes operations if their guard condition (`g`) is met.
    """

    def __init__(self):
        super().__init__('operation_runner')
        self.subscription = self.create_subscription(String, 'state_topic', self.state_callback, 10)
        self.client = self.create_client(Trigger, 'change_state')

        self.task_queue = []
        self.timer = self.create_timer(3.0, self.execute_operations)
        self.get_logger().info("Operation Runner initialized.")

    def state_callback(self, msg):
        """Receives state updates and extracts operations."""
        state_data = json.loads(msg.data)
        self.get_logger().info(f"Updated state: {state_data}")

        if "operations" in state_data:
            self.task_queue = state_data["operations"]

    def execute_operations(self):
        """Checks and executes operations if conditions are met."""
        if not self.task_queue:
            self.get_logger().info("No operations to execute.")
            return

        for operation in self.task_queue:
            if operation.get("g", False):  # Check guard condition
                self.get_logger().info(f"Executing operation: {operation['name']}")
                self.request_state_change()

    def request_state_change(self):
        """Requests a state change after executing an operation."""
        if not self.client.service_is_ready():
            self.get_logger().warn("State service unavailable.")
            return

        request = Trigger.Request()
        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        """Handles the response from the state manager."""
        try:
            response = future.result()
            self.get_logger().info(f"State updated: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    """
    Initializes and runs all nodes concurrently.
    """
    rclpy.init(args=args)

    state_manager = StateManager()
    control_node = ControlNode()
    monitoring_node = MonitoringNode()
    operation_runner = OperationRunner()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(state_manager)
    executor.add_node(control_node)
    executor.add_node(monitoring_node)
    executor.add_node(operation_runner)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        state_manager.destroy_node()
        control_node.destroy_node()
        monitoring_node.destroy_node()
        operation_runner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
