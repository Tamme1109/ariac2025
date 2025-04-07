import rclpy
from rclpy.node import Node

from ariac_msgs.srv import VacuumGripperControl, ChangeGripper
from ariac_msgs.msg import Part, PartPose
from geometry_msgs.msg import Pose, PoseStamped
from moveit import MoveItPy
from moveit.planning import PlanningComponent, PlanningSceneMonitor
from builtin_interfaces.msg import Duration
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_pose

from typing import List

class FloorRobot(Node):
    def __init__(self):
        super().__init__('floor_robot')

        # MoveIt setup
        self._moveit = MoveItPy(node_name="robotic_arm_moveit")
        self._floor_robot = self._moveit.get_planning_component("floor_robot")

        # Gripper service clients
        self._floor_gripper_enable = self.create_client(VacuumGripperControl, "/ariac/floor_robot/gripper/control")
        self._floor_tool_changer = self.create_client(ChangeGripper, "/ariac/floor_robot/change_gripper")

        # TF buffer for transforms
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.get_logger().info("RoboticArm node initialized")

    # ========== Gripper Control ==========

    def set_gripper_state(self, enable: bool) -> bool:
        if not self._floor_gripper_enable.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Gripper control service not available")
            return False

        req = VacuumGripperControl.Request()
        req.enable = enable
        future = self._floor_gripper_enable.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

    def change_gripper(self, gripper_type: int) -> bool:
        if not self._floor_tool_changer.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Tool changer service not available")
            return False

        req = ChangeGripper.Request()
        req.gripper_type = gripper_type
        future = self._floor_tool_changer.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

    # ========== Motion Planning ==========

    def move_to_pose(self, pose: Pose, world_frame: str = "world") -> bool:
        goal = PoseStamped()
        goal.pose = pose
        goal.header.frame_id = world_frame

        self._floor_robot.set_goal(goal)
        plan_result = self._floor_robot.plan()
        if not plan_result.success:
            self.get_logger().error("Planning failed")
            return False

        return self._floor_robot.execute()

    def move_to_joint_positions(self, joint_positions: List[float]) -> bool:
        joint_names = [
            "joint_1", "joint_2", "joint_3",
            "joint_4", "joint_5", "joint_6"
        ]
        joint_state = dict(zip(joint_names, joint_positions))

        self._floor_robot.set_joint_goal(joint_state)
        plan_result = self._floor_robot.plan()
        if not plan_result.success:
            self.get_logger().error("Joint planning failed")
            return False

        return self._floor_robot.execute()

    def move_home(self) -> bool:
        home_joints = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        return self.move_to_joint_positions(home_joints)
