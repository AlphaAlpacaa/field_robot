#!/usr/bin/env python3
"""
generate_field.py

Writes a Gazebo world containing rows of tomato plants with pickable fruit.

Plant geometry
--------------
Each plant is grown recursively: a trunk, then at every node either one
continuation or a fork into two, with length and radius shrinking each
generation and the direction deviating by a bounded angle while being pulled
slightly upward. That gives the uneven, bushy look of a real tomato plant --
one branch stops short, its sibling forks again -- without the regularity of
a fractal tree.

All of a plant's segments live in a single link as separate <visual> elements.
Separate links and joints would be more "correct" but cost far more to
simulate, and nothing here needs the branches to move independently.

Fruit hangs from a subset of branch tips, 2-4 per plant. Each tomato is its
own model held by a DetachableJoint: sending an empty message to its detach
topic releases it, which is the simulation equivalent of picking it. Without
that, a fruit resting on a branch would simply roll off.

Why plants are anchored instead of static
-----------------------------------------
A DetachableJoint needs a real (non-static) parent link, so each plant is a
dynamic model pinned to the world with a fixed joint.

Usage
-----
    python3 generate_field.py --output ../worlds/tomato_field.sdf
    python3 generate_field.py --rows 4 --plants-per-row 8
    python3 generate_field.py --rows 2 --plants-per-row 4   # lighter
"""

import argparse
import math
import random

# ----------------------------------------------------------------------------
# plant proportions (metres). Roughly a staked indeterminate tomato.
# ----------------------------------------------------------------------------
# A tomato plant branches low and spreads sideways; it is not a little tree.
# Hence the short trunk and the wide first-generation angle.
TRUNK_LENGTH = 0.20
TRUNK_RADIUS = 0.022

LENGTH_RATIO = 0.82          # how much shorter each generation is
RADIUS_RATIO = 0.68          # how much thinner
MIN_RADIUS = 0.004           # stop branching below this
MAX_DEPTH = 4                # 4 generations -> 3-5 nodes along a path

FORK_CHANCE = 0.6            # probability a node splits in two
BRANCH_ANGLE = (0.45, 0.95)  # radians away from the parent direction
UPWARD_BIAS = 0.22           # 0 = keep deviating freely, 1 = snap to vertical

LEAF_CHANCE = 0.6            # chance a mid-level node carries a leaf cluster
LEAF_RADIUS = 0.055

FRUIT_RADIUS = 0.035
FRUIT_MASS = 0.12
FRUIT_MIN_DEPTH = 2          # no fruit on the trunk itself


# ----------------------------------------------------------------------------
# small vector helpers (pure python; the geometry here is too simple for numpy)
# ----------------------------------------------------------------------------
def normalise(v):
    n = math.sqrt(sum(c * c for c in v))
    return (v[0] / n, v[1] / n, v[2] / n)


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def rotate_about(v, axis, angle):
    """Rodrigues' rotation formula."""
    k = normalise(axis)
    c, s = math.cos(angle), math.sin(angle)
    kv = cross(k, v)
    kdotv = sum(a * b for a, b in zip(k, v))
    return tuple(v[i] * c + kv[i] * s + k[i] * kdotv * (1 - c) for i in range(3))


def perpendicular(v):
    """Any unit vector perpendicular to v."""
    ref = (0.0, 0.0, 1.0) if abs(v[2]) < 0.9 else (1.0, 0.0, 0.0)
    return normalise(cross(v, ref))


def segment_pose(p0, p1):
    """
    Pose for a cylinder spanning p0..p1.

    A cylinder's axis runs along z, so the rotation only needs a pitch away
    from vertical and a yaw around it; roll is meaningless for a cylinder.
    """
    d = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])
    length = math.sqrt(sum(c * c for c in d))
    mid = ((p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2, (p0[2] + p1[2]) / 2)
    yaw = math.atan2(d[1], d[0])
    pitch = math.atan2(math.hypot(d[0], d[1]), d[2])
    return mid, (0.0, pitch, yaw), length


# ----------------------------------------------------------------------------
# plant growth
# ----------------------------------------------------------------------------
def grow(rng):
    """
    Build one plant.

    Returns (segments, leaves, tips) where
      segments: list of (p0, p1, radius)
      leaves:   list of (position, radius)
      tips:     list of (position, depth) - candidate fruit sites
    """
    segments, leaves, tips = [], [], []

    def branch(start, direction, length, radius, depth):
        end = (start[0] + direction[0] * length,
               start[1] + direction[1] * length,
               start[2] + direction[2] * length)
        segments.append((start, end, radius))

        if depth >= FRUIT_MIN_DEPTH:
            tips.append((end, depth))

        if depth >= 1 and rng.random() < LEAF_CHANCE:
            leaves.append((end, LEAF_RADIUS * (RADIUS_RATIO ** (depth - 1))))

        if depth >= MAX_DEPTH or radius * RADIUS_RATIO < MIN_RADIUS:
            return

        n_children = 2 if rng.random() < FORK_CHANCE else 1
        for _ in range(n_children):
            angle = rng.uniform(*BRANCH_ANGLE)
            # rotate away from the parent direction about a random perpendicular
            axis = rotate_about(perpendicular(direction), direction,
                                rng.uniform(0, 2 * math.pi))
            new_dir = rotate_about(direction, axis, angle)
            # pull back toward vertical: branches rise, they do not droop
            new_dir = normalise((new_dir[0],
                                 new_dir[1],
                                 new_dir[2] + UPWARD_BIAS))
            branch(end,
                   new_dir,
                   length * LENGTH_RATIO * rng.uniform(0.85, 1.15),
                   radius * RADIUS_RATIO,
                   depth + 1)

    # the trunk leans a few degrees off vertical so the plants are not clones
    trunk_dir = normalise((rng.uniform(-0.12, 0.12),
                           rng.uniform(-0.12, 0.12),
                           1.0))
    branch((0.0, 0.0, 0.0), trunk_dir, TRUNK_LENGTH, TRUNK_RADIUS, 0)
    return segments, leaves, tips


# ----------------------------------------------------------------------------
# SDF emitters
# ----------------------------------------------------------------------------
def header(world_name):
    return f'''<?xml version="1.0" ?>
<!--
  Generated by scripts/generate_field.py - do not edit by hand.
  Re-run the script to change the layout.
-->
<sdf version="1.9">
  <world name="{world_name}">

    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system"
            name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-contact-system"
            name="gz::sim::systems::Contact"/>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.6 0.75 0.9 1</background>
      <sky/>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.95 0.92 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.3 -0.9</direction>
    </light>

    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal><size>120 120</size></plane>
          </geometry>
          <surface>
            <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal><size>120 120</size></plane>
          </geometry>
          <material>
            <ambient>0.33 0.24 0.16 1</ambient>
            <diffuse>0.42 0.30 0.19 1</diffuse>
            <specular>0.02 0.02 0.02 1</specular>
          </material>
        </visual>
      </link>
    </model>
'''


def soil_bed(row_index, y, length, centre_x, width=0.55):
    """
    A slightly raised soil ridge under each row.

    Visual only, deliberately: with a collision box the ridge becomes a 6 cm
    kerb that the wheels catch on, and a robot spawned over one ends up
    straddling it and going nowhere. Real beds are loose soil the machine
    drives straight over, so leaving the collision out is closer to the truth
    than modelling it as a solid block.
    """
    return f'''
    <model name="bed_{row_index}">
      <static>true</static>
      <pose>{centre_x:.3f} {y:.3f} 0.03 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>{length:.2f} {width} 0.06</size></box></geometry>
          <material>
            <ambient>0.22 0.15 0.10 1</ambient>
            <diffuse>0.28 0.19 0.12 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
'''


def plant_model(name, x, y, yaw, segments, leaves, fruit_specs,
                collision_radius=0.010):
    """
    One plant as a single link holding every branch segment.

    Only the thicker segments get collision geometry. Twigs a few millimetres
    across would add dozens of contact shapes per plant for no benefit - the
    arm should be able to push through foliage, and the physics engine has
    better things to do.
    """
    parts = []

    for (p0, p1, radius) in segments:
        mid, rpy, length = segment_pose(p0, p1)
        shade = min(0.52, 0.22 + radius * 6)      # thinner twigs look greener
        parts.append(f'''
        <visual name="branch_{len(parts)}">
          <pose>{mid[0]:.4f} {mid[1]:.4f} {mid[2]:.4f} {rpy[0]:.4f} {rpy[1]:.4f} {rpy[2]:.4f}</pose>
          <geometry>
            <cylinder><radius>{radius:.4f}</radius><length>{length:.4f}</length></cylinder>
          </geometry>
          <material>
            <ambient>{shade*0.5:.3f} {shade:.3f} {shade*0.42:.3f} 1</ambient>
            <diffuse>{shade*0.6:.3f} {shade*1.25:.3f} {shade*0.5:.3f} 1</diffuse>
          </material>
        </visual>''')

        if radius >= collision_radius:
            parts.append(f'''
        <collision name="branch_col_{len(parts)}">
          <pose>{mid[0]:.4f} {mid[1]:.4f} {mid[2]:.4f} {rpy[0]:.4f} {rpy[1]:.4f} {rpy[2]:.4f}</pose>
          <geometry>
            <cylinder><radius>{radius:.4f}</radius><length>{length:.4f}</length></cylinder>
          </geometry>
        </collision>''')

    for i, (pos, radius) in enumerate(leaves):
        parts.append(f'''
        <visual name="leaf_{i}">
          <pose>{pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f} 0 0 0</pose>
          <geometry><sphere><radius>{radius:.4f}</radius></sphere></geometry>
          <material>
            <ambient>0.13 0.30 0.10 1</ambient>
            <diffuse>0.19 0.45 0.14 1</diffuse>
            <specular>0.04 0.07 0.04 1</specular>
          </material>
          <transparency>0.2</transparency>
        </visual>''')

    detachables = ''.join(f'''
      <plugin filename="gz-sim-detachable-joint-system"
              name="gz::sim::systems::DetachableJoint">
        <parent_link>stem</parent_link>
        <child_model>{fruit_name}</child_model>
        <child_link>fruit</child_link>
        <detach_topic>/{fruit_name}/detach</detach_topic>
      </plugin>''' for fruit_name, _, _ in fruit_specs)

    return f'''
    <model name="{name}">
      <pose>{x:.3f} {y:.3f} 0 0 0 {yaw:.3f}</pose>

      <!-- anchor to the world: keeps the plant upright but leaves it a
           dynamic model, which DetachableJoint requires -->
      <joint name="anchor" type="fixed">
        <parent>world</parent>
        <child>stem</child>
      </joint>

      <link name="stem">
        <inertial>
          <pose>0 0 0.25 0 0 0</pose>
          <mass>1.2</mass>
          <inertia><ixx>0.05</ixx><iyy>0.05</iyy><izz>0.01</izz>
                   <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
{''.join(parts)}
      </link>
{detachables}
    </model>
'''


def fruit_model(name, x, y, z, ripe=True, scale=1.0):
    """A single tomato: its own dynamic model so it can be detached and carried."""
    if ripe:
        ambient, diffuse = "0.45 0.05 0.03 1", "0.80 0.10 0.06 1"
    else:
        ambient, diffuse = "0.30 0.35 0.10 1", "0.55 0.62 0.18 1"

    radius = FRUIT_RADIUS * scale
    mass = FRUIT_MASS * scale ** 3
    inertia = 2.0 / 5.0 * mass * radius ** 2

    return f'''
    <model name="{name}">
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
      <link name="fruit">
        <inertial>
          <mass>{mass:.4f}</mass>
          <inertia><ixx>{inertia:.6f}</ixx><iyy>{inertia:.6f}</iyy><izz>{inertia:.6f}</izz>
                   <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="collision">
          <geometry><sphere><radius>{radius:.4f}</radius></sphere></geometry>
          <surface>
            <friction><ode><mu>1.5</mu><mu2>1.5</mu2></ode></friction>
            <contact><ode><min_depth>0.002</min_depth><kp>100000</kp></ode></contact>
          </surface>
        </collision>
        <visual name="visual">
          <geometry><sphere><radius>{radius:.4f}</radius></sphere></geometry>
          <material>
            <ambient>{ambient}</ambient>
            <diffuse>{diffuse}</diffuse>
            <specular>0.3 0.3 0.3 1</specular>
          </material>
        </visual>
      </link>
    </model>
'''


def rotate_xy(px, py, yaw):
    """Plant-local coordinates -> world, for the plant's own yaw."""
    c, s = math.cos(yaw), math.sin(yaw)
    return px * c - py * s, px * s + py * c


# ----------------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--output', default='tomato_field.sdf')
    p.add_argument('--world-name', default='tomato_field')
    p.add_argument('--rows', type=int, default=3)
    p.add_argument('--plants-per-row', type=int, default=6)
    p.add_argument('--row-spacing', type=float, default=1.8,
                   help='distance between rows (m)')
    p.add_argument('--plant-spacing', type=float, default=0.65,
                   help='distance between plants within a row (m)')
    p.add_argument('--min-fruits', type=int, default=2)
    p.add_argument('--max-fruits', type=int, default=4)
    p.add_argument('--unripe-ratio', type=float, default=0.3,
                   help='fraction of fruit that is green (0-1)')
    p.add_argument('--start-x', type=float, default=2.0,
                   help='distance from the origin to the first plant (m)')
    p.add_argument('--seed', type=int, default=42)
    args = p.parse_args()

    rng = random.Random(args.seed)

    sdf = [header(args.world_name)]
    row_length = args.plants_per_row * args.plant_spacing

    n_plants = n_fruits = n_segments = 0

    for r in range(args.rows):
        y_row = (r - (args.rows - 1) / 2.0) * args.row_spacing

        bed_length = row_length + 0.6
        bed_centre_x = args.start_x - 0.3 + bed_length / 2.0
        sdf.append(soil_bed(r, y_row, bed_length, bed_centre_x))

        for c in range(args.plants_per_row):
            x = args.start_x + c * args.plant_spacing
            jx = x + rng.uniform(-0.04, 0.04)
            jy = y_row + rng.uniform(-0.05, 0.05)
            yaw = rng.uniform(0, 2 * math.pi)

            segments, leaves, tips = grow(rng)
            n_segments += len(segments)

            # fruit goes on a random subset of tips, preferring the outer ones
            tips.sort(key=lambda t: -t[1])
            wanted = rng.randint(args.min_fruits, args.max_fruits)
            chosen = rng.sample(tips[:max(wanted * 3, wanted)],
                                min(wanted, len(tips)))

            plant_name = f'plant_{r}_{c}'
            fruit_specs = []
            fruit_blocks = []

            for f, (tip, _depth) in enumerate(chosen):
                fruit_name = f'tomato_{r}_{c}_{f}'
                scale = rng.uniform(0.85, 1.15)
                # hangs just under the tip of its branch
                lx, ly = tip[0], tip[1]
                wx, wy = rotate_xy(lx, ly, yaw)
                fx = jx + wx
                fy = jy + wy
                fz = tip[2] - FRUIT_RADIUS * scale - 0.012
                ripe = rng.random() > args.unripe_ratio

                fruit_specs.append((fruit_name, (fx, fy, fz), ripe))
                fruit_blocks.append(fruit_model(fruit_name, fx, fy, fz,
                                                ripe, scale))
                n_fruits += 1

            sdf.append(plant_model(plant_name, jx, jy, yaw,
                                   segments, leaves, fruit_specs))
            sdf.extend(fruit_blocks)
            n_plants += 1

    sdf.append('\n  </world>\n</sdf>\n')

    with open(args.output, 'w') as fh:
        fh.write(''.join(sdf))

    print(f'wrote {args.output}')
    print(f'  {args.rows} rows x {args.plants_per_row} plants = {n_plants} plants')
    print(f'  {n_segments} branch segments ({n_segments/max(n_plants,1):.1f} per plant)')
    print(f'  {n_fruits} fruits')
    print(f'  field spans x=[{args.start_x:.1f}, {args.start_x + row_length:.1f}], '
          f'y=[{-(args.rows-1)/2*args.row_spacing:.1f}, '
          f'{(args.rows-1)/2*args.row_spacing:.1f}]')
    print()
    print('pick a fruit from the command line with:')
    print('  gz topic -t /tomato_0_0_0/detach -m gz.msgs.Empty -p ""')


if __name__ == '__main__':
    main()
