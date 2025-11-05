#!/usr/bin/env python3

"""
Controls the robotic arm for pick-and-place tasks using MoveIt2.

Author: Elena Oikonomou (adapted to ROS2)
Date:   Fall 2023 / 2025
"""

import numpy as np
import random
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import GetPositionIK
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from typing import Tuple
from tf_transformations import quaternion_from_euler

from pick_and_place_msgs.msg import DetectedObjectsStamped, DetectedObject

try:
    from moveit.planning import MoveItPy, PlanningComponent
    from moveit.core.robot_state import RobotState
    MOVEIT_PY_AVAILABLE = True
except ImportError:
    MOVEIT_PY_AVAILABLE = False
    print("Warning: MoveItPy not available, using alternative approach")


class Controller(Node):
    def __init__(self):
        super().__init__('panda_controller')
        
        self.red_bin = (-0.5, -0.25)
        self.green_bin = (-0.5, 0.0)
        self.blue_bin = (-0.5, 0.25)
        
        self.workbench_height = 0.2
        self.x_offset = 0.01
        self.z_offset = 0.105
        self.z_pre_pick_offset = 0.2
        self.z_pre_place_offset = 0.2
        
        self.objects_on_workbench = []
        self.current_joint_states = None
        
        # Joint names for Panda
        self.joint_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
            'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]
        
        # Home/neutral position
        self.neutral_pose = [0.0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785]
        
        # Publishers
        self.joint_pub = self.create_publisher(
            JointTrajectory,
            '/panda_arm_controller/joint_trajectory',
            10)
        
        self.planning_scene_pub = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10)
        
        # Subscribers
        self.create_subscription(
            DetectedObjectsStamped,
            '/object_detection',
            self.update_objects_callback,
            10)
        
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        # Gripper action client
        self.gripper_action_client = ActionClient(
            self,
            GripperCommand,
            '/panda_gripper/gripper_cmd')
        
        # Wait for action server
        self.get_logger().info('Waiting for gripper action server...')
        self.gripper_action_client.wait_for_server()
        
        # Add collision objects
        self.get_logger().info('Adding collision objects to planning scene...')
        self.create_timer(2.0, self.add_collision_objects_once)
        
    def add_collision_objects_once(self):
        """Add collision objects once after a delay."""
        self.add_collision_objects()
        # Cancel the timer after first execution
        for timer in self.timers:
            timer.cancel()
    
    def joint_state_callback(self, msg: JointState) -> None:
        """Update current joint states."""
        self.current_joint_states = msg
    
    def update_objects_callback(self, msg: DetectedObjectsStamped) -> None:
        """Updates the objects that are currently on top of the workbench."""
        self.objects_on_workbench = msg.detected_objects
    
    def select_random_object(self) -> DetectedObject:
        """Selects an object at random from the ones that are currently on top of the workbench."""
        return random.choice(self.objects_on_workbench)
    
    def are_objects_on_workbench(self) -> bool:
        """Checks whether there are any objects on top of the workbench."""
        return len(self.objects_on_workbench) > 0
    
    def move_object(self, obj: DetectedObject) -> None:
        """Picks the given object and places it to the correct color bin."""
        x = obj.x_world
        y = obj.y_world
        z = obj.height + self.workbench_height
        color = obj.color
        
        self.get_logger().info(f"\nSelected Object: {color}    (x,y) = ({x:.3f}, {y:.3f})\n")
        
        self.pick(x=x, y=y, z=z)
        bin_pos = self.select_bin(color)
        self.place(x=bin_pos[0], y=bin_pos[1], z=0.5)
    
    def select_bin(self, color: str) -> Tuple[float, float]:
        """Returns the (x,y) position in the world frame of the given color bin."""
        if color == "red":
            return self.red_bin
        elif color == "green":
            return self.green_bin
        elif color == "blue":
            return self.blue_bin
        else:
            self.get_logger().info('The object color does not match an available bin color. Using green bin.')
            return self.green_bin
    
    def pick(self, x: float, y: float, z: float, roll: float=0, pitch: float=np.pi, yaw: float=0, object_width: float=0.025) -> None:
        """Picks up the object at the given position with the given end-effector orientation."""
        pre_pick_position = np.array([x + self.x_offset, y, z + self.z_offset + self.z_pre_pick_offset])
        pick_position = np.array([x + self.x_offset, y, z + self.z_offset])
        pick_orientation = quaternion_from_euler(roll, pitch, yaw)
        
        # Pre-pick
        self.move_to_cartesian_pose(pos=pre_pick_position, ori=pick_orientation)
        self.gripper_open()
        
        # Pick
        self.move_to_cartesian_pose(pos=pick_position, ori=pick_orientation)
        self.gripper_close(width=object_width)
        
        # Post-pick
        self.move_to_cartesian_pose(pos=pre_pick_position, ori=pick_orientation)
    
    def place(self, x: float, y: float, z: float, roll: float=0, pitch: float=np.pi, yaw: float=0) -> None:
        """Places the object at the given position with the given end-effector orientation."""
        pre_place_position = np.array([x, y, z + self.z_pre_place_offset])
        place_position = np.array([x, y, z])
        place_orientation = quaternion_from_euler(roll, pitch, yaw)
        
        # Pre-place
        self.move_to_cartesian_pose(pos=pre_place_position, ori=place_orientation)
        
        # Place
        self.move_to_cartesian_pose(pos=place_position, ori=place_orientation)
        self.gripper_open()
        
        # Post-place
        self.move_to_cartesian_pose(pos=pre_place_position, ori=place_orientation)
    
    def move_to_neutral(self) -> None:
        """Move robot to neutral position."""
        self.move_to_joint_positions(self.neutral_pose)
    
    def move_to_joint_positions(self, joint_positions: list) -> None:
        """Move to specified joint positions."""
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = 2
        point.time_from_start.nanosec = 0
        
        trajectory.points.append(point)
        self.joint_pub.publish(trajectory)
        
        # Wait for motion to complete
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=2.5))
    
    def move_to_cartesian_pose(self, pos: np.ndarray, ori: tuple) -> None:
        """Move to a cartesian pose (position + orientation)."""
        # Simplified: In a full implementation, this would use MoveIt2 for IK and planning
        # For now, we'll log the desired pose
        self.get_logger().info(f'Moving to position: {pos}, orientation: {ori}')
        
        # In a real implementation, you would:
        # 1. Call MoveIt2 IK service to convert cartesian pose to joint angles
        # 2. Plan a trajectory to those joint angles
        # 3. Execute the trajectory
        
        # Placeholder for actual motion
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1.0))
    
    def gripper_open(self) -> None:
        """Open the gripper."""
        goal = GripperCommand.Goal()
        goal.command.position = 0.04  # Open position
        goal.command.max_effort = 5.0
        
        self.get_logger().info('Opening gripper...')
        future = self.gripper_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.5))
    
    def gripper_close(self, width: float = 0.01) -> None:
        """Close the gripper to grasp an object."""
        goal = GripperCommand.Goal()
        goal.command.position = width
        goal.command.max_effort = 5.0
        
        self.get_logger().info('Closing gripper...')
        future = self.gripper_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.5))
    
    def add_collision_objects(self) -> None:
        """Adds objects in the scene so the MoveIt planner can avoid collisions."""
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        
        # Add workbench
        workbench = CollisionObject()
        workbench.header.frame_id = "world"
        workbench.id = "workbench"
        
        workbench_primitive = SolidPrimitive()
        workbench_primitive.type = SolidPrimitive.BOX
        workbench_primitive.dimensions = [1.0, 3.0, 0.2]
        
        workbench_pose = Pose()
        workbench_pose.position.x = 0.7
        workbench_pose.position.y = 0.0
        workbench_pose.position.z = 0.1
        workbench_pose.orientation.w = 1.0
        
        workbench.primitives.append(workbench_primitive)
        workbench.primitive_poses.append(workbench_pose)
        workbench.operation = CollisionObject.ADD
        
        # Add bin bench
        binbench = CollisionObject()
        binbench.header.frame_id = "world"
        binbench.id = "binbench"
        
        binbench_primitive = SolidPrimitive()
        binbench_primitive.type = SolidPrimitive.BOX
        binbench_primitive.dimensions = [0.4, 1.5, 0.2]
        
        binbench_pose = Pose()
        binbench_pose.position.x = -0.55
        binbench_pose.position.y = 0.0
        binbench_pose.position.z = 0.1
        binbench_pose.orientation.w = 1.0
        
        binbench.primitives.append(binbench_primitive)
        binbench.primitive_poses.append(binbench_pose)
        binbench.operation = CollisionObject.ADD
        
        planning_scene.world.collision_objects.append(workbench)
        planning_scene.world.collision_objects.append(binbench)
        
        self.planning_scene_pub.publish(planning_scene)
        self.get_logger().info('Collision objects added to planning scene')


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

