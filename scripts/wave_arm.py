#!/usr/bin/env python3
"""
wave_arm.py
Minimal sanity check for the arm: sends a few joint positions through the
JointTrajectoryController, without MoveIt in the loop.

Run the simulation first, then:
    ros2 run field_robot wave_arm.py
or simply:
    python3 wave_arm.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

JOINTS = [
    'arm_shoulder_pan_joint',
    'arm_shoulder_lift_joint',
    'arm_elbow_joint',
    'arm_wrist_1_joint',
    'arm_wrist_2_joint',
    'arm_wrist_3_joint',
]

# (positions, seconds from start)
WAYPOINTS = [
    ([0.0,  -1.57,  1.57, -1.57, -1.57, 0.0], 3),
    ([0.8,  -1.20,  1.20, -1.60, -1.57, 0.0], 6),
    ([-0.8, -1.20,  1.20, -1.60, -1.57, 0.0], 9),
    ([0.0,  -1.57,  1.57, -1.57, -1.57, 0.0], 12),
]


class ArmWaver(Node):

    def __init__(self):
        super().__init__('arm_waver')
        self.client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory')

    def send(self):
        self.get_logger().info('waiting for arm_controller action server...')
        if not self.client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error(
                'action server not available - is the simulation running '
                'and is arm_controller active? check: ros2 control list_controllers')
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        for positions, secs in WAYPOINTS:
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start = Duration(sec=secs)
            goal.trajectory.points.append(point)

        self.get_logger().info('sending trajectory')
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('goal rejected')
            return

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('done')


def main():
    rclpy.init()
    node = ArmWaver()
    node.send()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
