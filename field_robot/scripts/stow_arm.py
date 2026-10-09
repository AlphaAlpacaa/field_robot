#!/usr/bin/env python3
"""
stow_arm.py

Folds the arm over the back of the robot and opens the gripper.

Use it before driving and before scanning. In the default home pose the
forearm hangs over the front of the chassis, directly across the body
camera's view of the crop - and a high, extended arm is also what makes a
skid-steer tip over in a turn. Stowed, the mass sits low and inside the
wheelbase and the camera sees the plants.

    ros2 run field_robot stow_arm.py
    ros2 run field_robot stow_arm.py --ros-args -p pose:=home
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_JOINTS = [
    'arm_shoulder_pan_joint',
    'arm_shoulder_lift_joint',
    'arm_elbow_joint',
    'arm_wrist_1_joint',
    'arm_wrist_2_joint',
    'arm_wrist_3_joint',
]
GRIPPER_JOINTS = ['gripper_left_finger_joint', 'gripper_right_finger_joint']

POSES = {
    # folded over the back: out of the camera's way, low centre of mass
    'stow': [3.14, -2.0, 2.3, -1.9, -1.57, 0.0],
    # folded over the front: compact, but it blocks the body camera
    'home': [0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
    # reaching forward, for checking the workspace by eye
    'ready': [0.0, -1.0, 1.2, -1.7, -1.57, 0.0],
}


class Stower(Node):

    def __init__(self):
        super().__init__('stow_arm')
        self.declare_parameter('pose', 'stow')
        self.declare_parameter('seconds', 4)
        self.declare_parameter('open_gripper', True)

        self.arm = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory')
        self.grip = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory')

    def send(self, client, joints, positions, seconds):
        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('controller not available - is the '
                                    'simulation running?')
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joints
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = Duration(sec=seconds)
        goal.trajectory.points.append(pt)

        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error('goal rejected')
            return False
        rclpy.spin_until_future_complete(self, handle.get_result_async())
        return True


def main():
    rclpy.init()
    node = Stower()

    name = node.get_parameter('pose').value
    seconds = node.get_parameter('seconds').value

    if name not in POSES:
        node.get_logger().error(
            f'unknown pose "{name}"; try one of {", ".join(POSES)}')
    else:
        if node.get_parameter('open_gripper').value:
            node.send(node.grip, GRIPPER_JOINTS, [0.04, 0.04], 1)
        node.get_logger().info(f'moving to {name}')
        if node.send(node.arm, ARM_JOINTS, POSES[name], seconds):
            node.get_logger().info('done')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
