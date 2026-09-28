#!/usr/bin/env python3
"""
test_gripper.py
Opens and closes the gripper a couple of times to verify the control chain,
without MoveIt in the loop.

    ros2 run field_robot test_gripper.py
    ros2 run field_robot test_gripper.py 0.04    # just open to a given width
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

JOINTS = ['gripper_left_finger_joint', 'gripper_right_finger_joint']

OPEN = 0.04
CLOSED = 0.0


class GripperTester(Node):

    def __init__(self):
        super().__init__('gripper_tester')
        self.client = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory')

    def move(self, width, seconds=2):
        """width: 0.0 = closed, 0.04 = fully open"""
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'gripper_controller action server not found - check '
                '"ros2 control list_controllers"')
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = [width, width]
        point.time_from_start = Duration(sec=seconds)
        goal.trajectory.points.append(point)

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('goal rejected')
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info(f'moved to {width:.3f} m')
        return True


def main():
    rclpy.init()
    node = GripperTester()

    if len(sys.argv) > 1:
        node.move(float(sys.argv[1]))
    else:
        node.move(OPEN)
        node.move(CLOSED)
        node.move(OPEN)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
