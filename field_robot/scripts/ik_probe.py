#!/usr/bin/env python3
"""
ik_probe.py

Diagnostic: asks /compute_ik for a grid of poses in front of the robot, with
and without collision checking, and prints which ones have a solution.

The point is to separate two very different failures:

  * nothing solves even with collisions off  -> the poses are outside the
    arm's reachable workspace, or the IK link / frame names are wrong
  * solves with collisions off but not on    -> the arm can get there, but
    MoveIt thinks it would hit something. Usually a self-collision matrix
    that was generated from a different URDF, or a real clash with the
    chassis.

Run it with the simulation and moveit.launch.py already up:

    ros2 run field_robot ik_probe.py
"""

import math

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK

ERRORS = {
    1: 'SUCCESS',
    -1: 'FAILURE',
    -10: 'START_STATE_IN_COLLISION',
    -12: 'GOAL_IN_COLLISION',
    -31: 'NO_IK_SOLUTION',
    99999: 'no response',
}


def quat_from_approach(direction):
    z = _unit(direction)
    up = (0.0, 0.0, 1.0) if abs(z[2]) < 0.95 else (1.0, 0.0, 0.0)
    x = _unit(_cross(up, z))
    y = _cross(z, x)
    m = [[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]]
    t = m[0][0] + m[1][1] + m[2][2]
    if t > 0:
        s = 0.5 / math.sqrt(t + 1.0)
        return Quaternion(x=(m[2][1] - m[1][2]) * s, y=(m[0][2] - m[2][0]) * s,
                          z=(m[1][0] - m[0][1]) * s, w=0.25 / s)
    # the degenerate branches are not needed for the poses probed here
    return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)


def _unit(v):
    n = math.sqrt(sum(c * c for c in v)) or 1.0
    return (v[0] / n, v[1] / n, v[2] / n)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


class Probe(Node):

    def __init__(self):
        super().__init__('ik_probe')
        self.declare_parameter('ik_link', 'grasp_frame')
        self.declare_parameter('group', 'arm')
        self.link = self.get_parameter('ik_link').value
        self.group = self.get_parameter('group').value
        self.cli = self.create_client(GetPositionIK, '/compute_ik')

    def try_pose(self, x, y, z, direction, collisions):
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group
        req.ik_request.ik_link_name = self.link
        req.ik_request.avoid_collisions = collisions
        req.ik_request.robot_state = RobotState()
        req.ik_request.robot_state.is_diff = True
        req.ik_request.timeout = Duration(sec=1)

        ps = PoseStamped()
        ps.header.frame_id = 'base_footprint'
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation = quat_from_approach(direction)
        req.ik_request.pose_stamped = ps

        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=4.0)
        res = fut.result()
        return res.error_code.val if res else 99999


def main():
    rclpy.init()
    node = Probe()

    if not node.cli.wait_for_service(timeout_sec=15.0):
        node.get_logger().error('/compute_ik not available - '
                                'is moveit.launch.py running?')
        return

    print(f'\nprobing group={node.group} ik_link={node.link}, '
          f'frame=base_footprint\n')
    print(f'{"pose (x, y, z)":<22}{"approach":<12}'
          f'{"no collision check":<22}{"with collision check"}')
    print('-' * 78)

    forward = (1.0, 0.0, 0.0)
    down = (0.0, 0.0, -1.0)
    diagonal = _unit((1.0, 0.0, -1.0))

    tests = []
    for x in (0.35, 0.45, 0.55, 0.65, 0.75):
        tests.append((x, 0.0, 0.45, forward, 'forward'))
    for z in (0.35, 0.50, 0.65, 0.80):
        tests.append((0.55, 0.0, z, forward, 'forward'))
    for y in (-0.3, 0.0, 0.3):
        tests.append((0.5, y, 0.5, diagonal, 'diagonal'))
    tests.append((0.4, 0.0, 0.75, down, 'down'))
    tests.append((0.0, 0.45, 0.65, down, 'down'))

    free = blocked = 0
    for (x, y, z, direction, label) in tests:
        off = node.try_pose(x, y, z, direction, False)
        on = node.try_pose(x, y, z, direction, True)
        if off == 1:
            free += 1
        if on == 1:
            blocked += 1
        print(f'({x:5.2f},{y:6.2f},{z:5.2f})     {label:<12}'
              f'{ERRORS.get(off, off):<22}{ERRORS.get(on, on)}')

    print('-' * 78)
    print(f'{free}/{len(tests)} reachable ignoring collisions, '
          f'{blocked}/{len(tests)} with collision checking\n')

    if free == 0:
        print('Nothing is reachable at all. The IK link name, the planning '
              'group or the frame is probably wrong, not the geometry.')
    elif blocked < free:
        print('Reachable but blocked: MoveIt believes these poses collide. '
              'Check the self-collision matrix in the MoveIt config, and '
              'whether the arm really does clash with the chassis there.')
    else:
        print('IK looks healthy; the problem is elsewhere.')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
