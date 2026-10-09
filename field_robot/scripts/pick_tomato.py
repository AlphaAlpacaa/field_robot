#!/usr/bin/env python3
"""
pick_tomato.py

Picks tomatoes: choose a target from the camera, move the arm to it, close the
gripper, release the fruit from its plant, and drop it in the basket zone.

Two-stage perception
--------------------
The body camera scans at range and tells us which plant is worth approaching;
its position estimate is too coarse to grasp with (a tomato 4 m away is a
handful of pixels and its depth reading easily lands on a gap between leaves).
So the arm first moves to a LOOK pose near the coarse target, then the wrist
camera - centimetres away, filling the frame - supplies the position actually
used for the grasp. Stage one answers "where should I go", stage two answers
"where exactly is it".

Arm control
-----------
Rather than the MoveIt Python bindings, this talks to move_group over its
standard interfaces: /compute_ik turns a Cartesian grasp pose into joint
angles, and the MoveGroup action plans and executes a collision-free path to
those angles. Fewer moving parts, and the same calls work from any language.

Usage
-----
    ros2 run field_robot pick_tomato.py              # pick one and stop
    ros2 run field_robot pick_tomato.py --ros-args -p loop:=true
"""

import math
import subprocess

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseArray, PoseStamped, Point, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             PlanningOptions, RobotState)
from moveit_msgs.srv import GetPositionIK
from tf2_msgs.msg import TFMessage
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

GRIPPER_OPEN = 0.04
GRIPPER_CLOSED = 0.012     # a shade narrower than the fruit, so it squeezes

# Folded over the FRONT of the robot. Compact, but the forearm hangs in the
# body camera's line of sight, which is exactly where the fruit is.
HOME = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]

# Folded over the BACK instead: the shoulder is turned 180 degrees so the whole
# arm sits above the rear of the chassis. Nothing of it crosses the forward
# view, so the scan sees the crop rather than the robot's own elbow. This is
# also the sensible pose to drive in - a stowed arm keeps the centre of mass
# low and inside the wheelbase.
STOW = [3.14, -2.0, 2.3, -1.9, -1.57, 0.0]


# ---------------------------------------------------------------------------
# geometry helpers
# ---------------------------------------------------------------------------
def quat_from_approach(direction):
    """
    Orientation that points the gripper's approach axis (its local +z) along
    `direction`, keeping the fingers level.

    Build the rotation matrix column by column, then convert. The reference
    "up" vector fixes the roll, which is otherwise undetermined: any spin
    about the approach axis would satisfy the request.
    """
    z = _unit(direction)
    up = (0.0, 0.0, 1.0)
    if abs(z[2]) > 0.95:                       # approach is near-vertical
        up = (1.0, 0.0, 0.0)
    x = _unit(_cross(up, z))
    y = _cross(z, x)

    m = [[x[0], y[0], z[0]],
         [x[1], y[1], z[1]],
         [x[2], y[2], z[2]]]

    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        qx = (m[2][1] - m[1][2]) * s
        qy = (m[0][2] - m[2][0]) * s
        qz = (m[1][0] - m[0][1]) * s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = 2.0 * math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2])
        w = (m[2][1] - m[1][2]) / s
        qx = 0.25 * s
        qy = (m[0][1] + m[1][0]) / s
        qz = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = 2.0 * math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2])
        w = (m[0][2] - m[2][0]) / s
        qx = (m[0][1] + m[1][0]) / s
        qy = 0.25 * s
        qz = (m[1][2] + m[2][1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1])
        w = (m[1][0] - m[0][1]) / s
        qx = (m[0][2] + m[2][0]) / s
        qy = (m[1][2] + m[2][1]) / s
        qz = 0.25 * s

    return Quaternion(x=qx, y=qy, z=qz, w=w)


def _unit(v):
    n = math.sqrt(sum(c * c for c in v)) or 1.0
    return (v[0] / n, v[1] / n, v[2] / n)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def dist(p, q):
    return math.sqrt((p.x - q.x) ** 2 + (p.y - q.y) ** 2 + (p.z - q.z) ** 2)


# ---------------------------------------------------------------------------
class TomatoPicker(Node):

    def __init__(self):
        super().__init__('tomato_picker')

        self.declare_parameter('loop', False)
        self.declare_parameter('world', 'tomato_field')
        self.declare_parameter('approach_distance', 0.14)
        self.declare_parameter('look_distance', 0.30)
        self.declare_parameter('max_reach', 0.80)
        self.declare_parameter('min_reach', 0.25)
        self.declare_parameter('planning_time', 5.0)
        self.declare_parameter('velocity_scaling', 0.3)
        # where picked fruit is released, in base_footprint
        self.declare_parameter('basket_xyz', [-0.15, 0.30, 0.55])
        # Where the arm is bolted on, in base_footprint. Reach and approach
        # directions are measured from here, not from the middle of the robot:
        # a fruit 0.6 m ahead of the robot is only 0.65 m from the shoulder,
        # and backing off "towards the robot" from the robot's centre aims the
        # gripper straight into its own chassis.
        self.declare_parameter('arm_base_xyz', [-0.05, 0.0, 0.42])
        self.declare_parameter('robot_model_name', 'field_robot')
        self.declare_parameter('match_radius', 0.35)

        self.loop = self.get_parameter('loop').value
        self.world = self.get_parameter('world').value
        self.approach_d = self.get_parameter('approach_distance').value
        self.look_d = self.get_parameter('look_distance').value
        self.max_reach = self.get_parameter('max_reach').value
        self.min_reach = self.get_parameter('min_reach').value
        self.planning_time = self.get_parameter('planning_time').value
        self.vel_scale = self.get_parameter('velocity_scaling').value
        self.basket = self.get_parameter('basket_xyz').value
        ab = self.get_parameter('arm_base_xyz').value
        self.arm_base = Point(x=ab[0], y=ab[1], z=ab[2])
        self.robot_name = self.get_parameter('robot_model_name').value
        self.match_radius = self.get_parameter('match_radius').value

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=5)

        self.scan_poses = None
        self.pick_poses = None
        self.model_poses = {}

        self.create_subscription(PoseArray, '/tomatoes/scan_poses',
                                 self._on_scan, sensor_qos)
        self.create_subscription(PoseArray, '/tomatoes/pick_poses',
                                 self._on_pick, sensor_qos)
        self.create_subscription(TFMessage, '/model_poses',
                                 self._on_models, 10)

        self.ik = self.create_client(GetPositionIK, '/compute_ik')
        self.move = ActionClient(self, MoveGroup, '/move_action')
        self.grip = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory')

        self.picked = set()

    # -- subscriptions ------------------------------------------------------
    def _on_scan(self, msg):
        self.scan_poses = msg

    def _on_pick(self, msg):
        self.pick_poses = msg

    def _on_models(self, msg):
        # Keep whole transforms, not just positions: the robot's heading
        # matters as much as its location when converting a grasp point from
        # the robot's frame into the world.
        for t in msg.transforms:
            self.model_poses[t.child_frame_id] = t.transform

    # -- plumbing -----------------------------------------------------------
    def wait_for_everything(self):
        self.get_logger().info('waiting for move_group, IK and gripper...')
        if not self.ik.wait_for_service(timeout_sec=20.0):
            self.get_logger().error('/compute_ik not available - is '
                                    'moveit.launch.py running?')
            return False
        if not self.move.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('/move_action not available')
            return False
        if not self.grip.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('gripper controller not available')
            return False
        return True

    def spin_for(self, seconds):
        end = self.get_clock().now().nanoseconds + seconds * 1e9
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    # -- arm ----------------------------------------------------------------
    def solve_ik(self, position, orientation, attempts=3):
        """Cartesian pose -> joint angles, or None if unreachable."""
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'arm'
        req.ik_request.ik_link_name = 'grasp_frame'
        req.ik_request.avoid_collisions = True
        req.ik_request.robot_state = RobotState()
        req.ik_request.robot_state.is_diff = True
        req.ik_request.timeout = Duration(sec=1)

        ps = PoseStamped()
        ps.header.frame_id = 'base_footprint'
        ps.pose.position = position
        ps.pose.orientation = orientation
        req.ik_request.pose_stamped = ps

        for _ in range(attempts):
            future = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            res = future.result()
            if res and res.error_code.val == 1:
                name_to_pos = dict(zip(res.solution.joint_state.name,
                                       res.solution.joint_state.position))
                return [name_to_pos[j] for j in ARM_JOINTS]
        return None

    def move_arm_to(self, joint_values, label=''):
        """Plan and execute a collision-free path to these joint angles."""
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = 'arm'
        req.num_planning_attempts = 10
        req.allowed_planning_time = self.planning_time
        req.max_velocity_scaling_factor = self.vel_scale
        req.max_acceleration_scaling_factor = self.vel_scale

        constraints = Constraints()
        for name, value in zip(ARM_JOINTS, joint_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req.goal_constraints.append(constraints)

        goal.request = req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False

        send = self.move.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if not handle or not handle.accepted:
            self.get_logger().error(f'{label}: goal rejected')
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        code = result_future.result().result.error_code.val
        if code != 1:
            self.get_logger().error(f'{label}: motion failed (code {code})')
            return False
        self.get_logger().info(f'{label}: done')
        return True

    # -- gripper ------------------------------------------------------------
    def set_gripper(self, width, seconds=1):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = GRIPPER_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [width, width]
        pt.time_from_start = Duration(sec=seconds)
        goal.trajectory.points.append(pt)

        send = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if not handle or not handle.accepted:
            return False
        rclpy.spin_until_future_complete(self, handle.get_result_async())
        return True

    # -- fruit --------------------------------------------------------------
    def robot_transform(self):
        """The robot's pose in the world, or None with a readable complaint."""
        t = self.model_poses.get(self.robot_name)
        if t is not None:
            return t

        if not self.model_poses:
            self.get_logger().warn(
                'nothing arriving on /model_poses. Check that the bridge is '
                'running and that the world name in config/bridge.yaml matches '
                'the world actually loaded:  ros2 topic hz /model_poses')
        else:
            sample = sorted(n for n in self.model_poses
                            if not n.startswith('tomato_'))[:12]
            self.get_logger().warn(
                f'no pose for a model called "{self.robot_name}". '
                f'Names seen include: {", ".join(sample)}. '
                f'Set the right one with  -p robot_model_name:=<name>')
        return None

    def to_world(self, point, robot):
        """
        base_footprint -> world.

        The robot's heading has to be applied, not just its position: once it
        has turned to face along a row, a point half a metre "ahead" of the
        robot is nowhere near half a metre ahead in world coordinates. Only
        yaw is used - a wheeled base on flat ground does not roll or pitch
        enough to matter here.
        """
        q = robot.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        return Point(
            x=robot.translation.x + point.x * c - point.y * s,
            y=robot.translation.y + point.x * s + point.y * c,
            z=robot.translation.z + point.z)

    def detach_nearest(self, grasp_position):
        """
        Release whichever tomato is closest to the gripper.

        The detection pipeline never learns model names - it only sees pixels -
        so the fruit is identified here by matching the grasp point against the
        model poses coming from Gazebo.
        """
        robot = self.robot_transform()
        if robot is None:
            return None

        target = self.to_world(grasp_position, robot)

        best, best_d = None, 1e9
        for name, t in self.model_poses.items():
            if not name.startswith('tomato_') or name in self.picked:
                continue
            d = dist(target, Point(x=t.translation.x,
                                   y=t.translation.y,
                                   z=t.translation.z))
            if d < best_d:
                best, best_d = name, d

        if best is None:
            self.get_logger().warn('no unpicked tomato models on /model_poses')
            return None
        if best_d > self.match_radius:
            self.get_logger().warn(
                f'closest tomato is {best_d:.2f} m from the grasp point, '
                f'further than the {self.match_radius:.2f} m match radius - '
                f'not detaching anything')
            return None

        subprocess.run(['gz', 'topic', '-t', f'/{best}/detach',
                        '-m', 'gz.msgs.Empty', '-p', ''],
                       capture_output=True, timeout=5)
        self.picked.add(best)
        self.get_logger().info(f'detached {best} ({best_d:.3f} m from grasp)')
        return best

    # -- the sequence -------------------------------------------------------
    def choose_target(self):
        """Nearest coarse detection that is plausibly within the arm's reach."""
        if not self.scan_poses or not self.scan_poses.poses:
            return None
        candidates = []
        for pose in self.scan_poses.poses:
            d = dist(self.arm_base, pose.position)
            if self.min_reach < d < self.max_reach:
                candidates.append((d, pose.position))
        if not candidates:
            return None
        candidates.sort(key=lambda c: c[0])
        return candidates[0][1]

    def refine_target(self, coarse):
        """Prefer the wrist camera's view; fall back to the coarse estimate."""
        self.spin_for(1.0)                 # let a frame or two arrive
        if self.pick_poses and self.pick_poses.poses:
            best = min(self.pick_poses.poses,
                       key=lambda p: dist(coarse, p.position))
            if dist(coarse, best.position) < 0.25:
                self.get_logger().info('refined with the wrist camera')
                return best.position
        self.get_logger().info('wrist camera saw nothing; using the coarse fix')
        return coarse

    def candidate_directions(self, target):
        """
        Directions the gripper could come in along, best first.

        One direction is rarely enough: the straight-on one is often blocked
        by the plant, outside the arm's reach, or in a wrist singularity. So
        this offers a spread - swung left and right of straight-on, and tilted
        down from above - ordered from the most natural to the most contorted,
        which is how a real picking head works its way into a canopy.
        """
        dx = target.x - self.arm_base.x
        dy = target.y - self.arm_base.y
        yaw0 = math.atan2(dy, dx)

        for pitch in (0.0, 0.35, 0.7, -0.25):        # 0 = level, + = from above
            for dyaw in (0.0, 0.4, -0.4, 0.8, -0.8):
                yaw = yaw0 + dyaw
                yield ((math.cos(pitch) * math.cos(yaw),
                        math.cos(pitch) * math.sin(yaw),
                        -math.sin(pitch)),
                       (pitch, dyaw))

    def pose_along(self, target, direction, standoff):
        """The pose `standoff` metres back from the target along `direction`."""
        return (Point(x=target.x - direction[0] * standoff,
                      y=target.y - direction[1] * standoff,
                      z=target.z - direction[2] * standoff),
                quat_from_approach(direction))

    def plan_grasp(self, target):
        """
        Pick one direction that works for the whole grasp, not just one pose.

        Solving each waypoint on its own looks like it should work and does
        not: the approach might solve from the left and the grasp from above,
        and the arm then has to swing around the fruit between them, which the
        planner usually refuses and which would rake the gripper through the
        plant if it didn't. An approach pose only means anything if it is the
        grasp pose backed off along the same axis, so both are solved together
        and a direction is only accepted if it serves both.

        Returns (approach_joints, grasp_joints, direction) or None.
        """
        for direction, (pitch, dyaw) in self.candidate_directions(target):
            approach_pos, orientation = self.pose_along(
                target, direction, self.approach_d)
            grasp_pos, _ = self.pose_along(target, direction, 0.0)

            if approach_pos.z < 0.20 or grasp_pos.z < 0.20:
                continue                         # would drive into the ground

            approach_joints = self.solve_ik(approach_pos, orientation,
                                            attempts=1)
            if approach_joints is None:
                continue
            grasp_joints = self.solve_ik(grasp_pos, orientation, attempts=1)
            if grasp_joints is None:
                continue

            self.get_logger().info(
                f'grasp plan: pitch {math.degrees(pitch):.0f} '
                f'yaw {math.degrees(dyaw):+.0f}')
            return approach_joints, grasp_joints, direction

        self.get_logger().warn('no approach direction serves both the '
                               'approach and the grasp')
        return None

    def reach_pose(self, target, distance, label):
        """First direction that IK can solve for a single pose, or None."""
        for direction, (pitch, dyaw) in self.candidate_directions(target):
            position, orientation = self.pose_along(target, direction, distance)
            if position.z < 0.20:
                continue
            joints = self.solve_ik(position, orientation, attempts=1)
            if joints is not None:
                if pitch or dyaw:
                    self.get_logger().info(
                        f'{label}: pitch {math.degrees(pitch):.0f} '
                        f'yaw {math.degrees(dyaw):+.0f}')
                return joints
        self.get_logger().warn(f'{label}: no reachable approach direction')
        return None

    def stow(self):
        """Fold the arm out of the body camera's way."""
        return self.move_arm_to(STOW, 'stow')

    def pick_one(self):
        coarse = self.choose_target()
        if coarse is None:
            self.get_logger().info('nothing in reach - drive closer to a plant')
            return False
        self.get_logger().info(
            f'target at x={coarse.x:.2f} y={coarse.y:.2f} z={coarse.z:.2f}')

        # 1. look pose: back off so the wrist camera can see the fruit
        joints = self.reach_pose(coarse, self.look_d, 'look')
        if joints is None:
            return False
        if not self.set_gripper(GRIPPER_OPEN):
            return False
        if not self.move_arm_to(joints, 'look'):
            return False

        # 2. second look, up close
        fine = self.refine_target(coarse)

        # 3. approach and grasp, along one consistent axis
        plan = self.plan_grasp(fine)
        if plan is None:
            return False
        approach_joints, grasp_joints, direction = plan

        if not self.move_arm_to(approach_joints, 'approach'):
            return False
        if not self.move_arm_to(grasp_joints, 'grasp'):
            return False

        # 4. close, release from the plant, back out the way we came in
        self.set_gripper(GRIPPER_CLOSED)
        self.spin_for(0.4)
        if self.detach_nearest(fine) is None:
            self.set_gripper(GRIPPER_OPEN)
            return False
        self.spin_for(0.3)

        # Retreat along the approach axis rather than straight up: lifting
        # from inside the canopy drags the fruit through the branches above it.
        if not self.move_arm_to(approach_joints, 'retreat'):
            self.get_logger().warn('could not retreat cleanly')

        # 5. drop it in the basket
        basket = Point(x=self.basket[0], y=self.basket[1], z=self.basket[2])
        joints = self.reach_pose(basket, 0.0, 'basket')
        if joints is not None and self.move_arm_to(joints, 'basket'):
            self.set_gripper(GRIPPER_OPEN)
            self.spin_for(0.5)

        # back out of the camera's way, ready for the next scan
        self.stow()
        return True


def main():
    rclpy.init()
    node = TomatoPicker()

    if node.wait_for_everything():
        # Clear the view before trusting anything the body camera reports:
        # with the arm folded forwards it stares at its own elbow.
        node.get_logger().info('stowing the arm to clear the camera view')
        node.set_gripper(GRIPPER_OPEN)
        node.stow()

        node.get_logger().info('waiting for detections...')
        node.spin_for(2.5)

        while rclpy.ok():
            ok = node.pick_one()
            if not node.loop:
                break
            if not ok:
                node.stow()
                node.spin_for(2.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
