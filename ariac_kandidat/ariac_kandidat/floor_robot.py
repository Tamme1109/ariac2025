import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from ariac_msgs.srv import VacuumGripperControl, ChangeGripper
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
import moveit_commander
import sys


class FloorRobotNode(Node):
    def __init__(self):
        super().__init__('floor_robot_node')

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()

        # Service Clients
        self._floor_gripper_enable = self.create_client(VacuumGripperControl, '/floor_robot_gripper_control')
        self._change_gripper_client = self.create_client(ChangeGripper, '/floor_robot_change_gripper')

        self.get_logger().info("Floor Robot Node Initialized")

    def _move_floor_robot_to_pose(self, pose: PoseStamped):
        """ Move the robot to the specified pose """
        self.get_logger().info(f"Moving floor robot to pose: {pose.pose}")
        self._floor_robot.set_pose_target(pose.pose)
        success = self._floor_robot.go(wait=True)

        if success:
            self.get_logger().info("Move successful")
        else:
            self.get_logger().warn("Move failed")

    def floor_robot_move_joints_dict(self, dict_positions: dict):
        """ Move robot based on joint positions """
        new_joint_position = self._create_floor_joint_position_dict(dict_positions)
        with self._planning_scene_monitor.read_write() as scene:
            self._floor_robot.set_start_state(robot_state=scene.current_state)
            scene.current_state.joint_positions = new_joint_position
            self._floor_robot.set_goal_state(motion_plan_constraints=[])
        self._plan_and_execute()

    def floor_robot_move_to_joint_position(self, position_name: str):
        """ Move robot to a joint position from predefined set """
        with self._planning_scene_monitor.read_write() as scene:
            self._floor_robot.set_start_state(robot_state=scene.current_state)
            scene.current_state.joint_positions = self.floor_position_dict[position_name]
            self._floor_robot.set_goal_state(motion_plan_constraints=[])
        self._plan_and_execute()

    def set_floor_robot_gripper_state(self, state: bool):
        request = VacuumGripperControl.Request()
        request.enable = state
        future = self._floor_gripper_enable.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f'Gripper state changed: {state}')
        else:
            self.get_logger().warn('Failed to change gripper state')

    def _floor_robot_change_gripper(self, station: str, gripper_type: str):
        request = ChangeGripper.Request()
        request.gripper_type = ChangeGripper.Request.TRAY_GRIPPER if gripper_type == "trays" else ChangeGripper.Request.PART_GRIPPER
        future = self._change_gripper_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if not future.result().success:
            self.get_logger().error("Failed to change gripper")


def main(args=None):
    rclpy.init(args=args)
    node = FloorRobotNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
