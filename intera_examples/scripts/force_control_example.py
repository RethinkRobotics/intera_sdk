#! /usr/bin/env python
# Copyright (c) 2023, Rethink Robotics GmbH.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""This script is illustrating the force control behavior via the motion controller."""

import rospy
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions,
    InteractionOptions
)
from intera_motion_msgs.msg import TrajectoryOptions
from intera_interface import Limb

from geometry_msgs.msg import (
    PoseStamped
)

import numpy as np


def position_from_pose(pose):
    return np.array([pose.position.x, pose.position.y, pose.position.z])


rospy.init_node('force_control_example')

limb = Limb()

start_angles = [-0.0033271484375, -0.9191728515625, 0.0018935546875,
                1.8378505859375, 0.0025751953125, 0.6456650390625, -0.0028447265625]

# Pose for start angles: should be something above a surface, e.g. a table.
# - Translation: [0.565, 0.159, 0.180]
# - Rotation: in Quaternion [0.767, -0.642, 0.001, -0.000]
#            in RPY (radian) [-3.140, -0.001, -1.393]
#            in RPY (degree) [-179.928, -0.058, -79.838]


wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=0.2,
                                 max_joint_accel=0.5)

# In this case, we would like to change the pose of the hand camera by 90°
# about the x-dir.
endpoint = 'right_hand'

# Get the current state (pose, twist, wrench) of the right_hand
state = limb.tip_state(endpoint)
pose = state.pose

print('Going to initial pose.')
# Setup a new pose with the original position and  orientation
pose_new = PoseStamped()
pose_new.pose.position = pose.position
pose_new.pose.orientation = pose.orientation


pre_position = MotionWaypoint(options=wpt_opts.to_msg(), limb=limb,
                              active_endpoint=endpoint)

# Set the pose of the waypoint to newly create pose.
# Make sure to set the endpoint otherwise this will be overwritten
pre_position.set_cartesian_pose(pose_new, active_endpoint=endpoint)
pre_position.set_joint_angles(start_angles)

# Create a new standard trajectory to go into a pre position
traj_opts = TrajectoryOptions()
traj_opts.interpolation_type = 'JOINT'
standard_motion = MotionTrajectory(trajectory_options=traj_opts, limb=limb)
# Set the waypoint options
standard_motion.append_waypoint(pre_position)
# # Execute the trajectory
standard_motion.send_trajectory()
end_state = limb.tip_state(endpoint)

print('Next motion with a force command')
rospy.sleep(rospy.Duration(2))

# Create a new goal which should be in contact with a surface:
goal = PoseStamped()
goal.pose = end_state.pose
goal.pose.position.z -= 0.08
# goal might even be below the surface, this is ok since we are going to use force control!

# In the following, we create the interaction options for our example.
inter_opts = InteractionOptions()
inter_opts.set_force_command([0, 0, -10, 0, 0, 0])  # 10 N force command in -z-direction
# by default this is in base frame coordinates
# (force, torque); if set_interaction_frame is used, the force command is in defined in the
# interaction frame.
# inter_opts.set_interaction_frame(goal.pose), e.g. the goal pose; changing the interfaction frames
# affects the force command, which is now using the interaction frame as a base frame.
inter_opts.set_disable_damping_in_force_control(True)
# Interaction control mode per Cartesian degree of freedom [x, y, z, rot_x, rot_y, rot_z] (6D):
inter_opts.set_interaction_control_mode(
    [InteractionOptions.impedance_mode,    # force x-dir.
     InteractionOptions.impedance_mode,    # force y-dir.
     InteractionOptions.force_limit_mode,  # force z-dir.
     InteractionOptions.impedance_mode,    # torque x-dir.
     InteractionOptions.impedance_mode,    # torque y-dir.
     InteractionOptions.impedance_mode])   # torque z-dir.

# InteractionOptions.force_limit_mode:
# This means we use a force command in this direction. In addition, the motion is stopped,
# if the distance to active waypoint is bigger than 0.12m or 0.2618 rad. See command,
# rosparam get /motor_control_plugins/InteractionController/forceMotionBounds
# for the limits for the force_limit mode, we will a tuple/list of the bounds, as
# [bound_pos (m), bound_orientation (rad), bound_vel_linear (m/s), bound_vel_angular (rad/s)]

# If you want to allow, arbitrary displacements, use InteractionOptions.force_mode instead.

# All modes: (InteractionOptions.impedance_mode, InteractionOptions.impedance_limit_mode,
# InteractionOptions.force_mode, InteractionOptions.force_limit_mode)

# For impedance modes you can also play with the stiffness values. K_impedance allows deviations
# from the interation
# inter_opts.set_K_impedance([1300, 1300, 1300, 30, 30, 30])  # uncomment to test it
# N/m for first three and Nm/rad for the rest
# inter_opts.set_K_nullspace([5.0, 10.0, 5.0, 10.0, 5.0, 10.0, 5.0])  # Nm/rad; uncomment to test it

# Note, that the force sensing of Sawyer is not 100% accurate, and forces around 5N cannot be
# accurately measured. Besids, the TCP forces may have an offset about 5N. Thus, if you want the TCP
# to only move in z-direction it is best, to set set_interaction_control_mode to impedance in all
# other directions, otherwise undesired drifts can occur. Alternatively, working only with impedance
# and different stiffness in different direction is also possible, and I suggest experimenting with
# it.

force_ctrl_opts = TrajectoryOptions()
force_ctrl_opts.interaction_params = inter_opts.to_msg()
force_ctrl_opts.interaction_control = True
force_ctrl_opts.interpolation_type = 'CARTESIAN'
# A new waypoint for the force controlled trajectory
wpt_opts.set_max_linear_speed(0.3)
force_wpt = MotionWaypoint(options=wpt_opts.to_msg(), limb=limb,
                           active_endpoint=endpoint)
force_wpt.set_cartesian_pose(goal, active_endpoint=endpoint)
force_ctrl_motion = MotionTrajectory(trajectory_options=force_ctrl_opts, limb=limb)
force_ctrl_motion.append_waypoint(force_wpt)
force_ctrl_motion.send_trajectory()

start = rospy.Time.now()
interaction_time = rospy.Duration(10)
time_active = rospy.Time.now() - start
np.set_printoptions(precision=3)
while time_active < interaction_time:
    time_active = rospy.Time.now() - start
    dt = interaction_time - time_active
    print('Interation control for additional {0:.1f} s ; try pushing the TCP away'.format(
        dt.to_sec()))
    end_state = limb.tip_state(endpoint)
    dx = position_from_pose(end_state.pose) - position_from_pose(goal.pose)
    print('Distance to goal: {}'.format(dx))
    force = limb.endpoint_effort()['force']
    print('TCP forces: [{force.x:.1f}, {force.y:.1f}, {force.z:.1f}]'.format(force=force))
    rospy.Rate(2).sleep()

print('Stopping interaction control and going to the initial pre-position!')
# If we use an unlimited interaction mode, we should stop it at some point, otherwise the force/
# impedance controllers will still be active. You can stop it, e.g., by sending a normal motion
# command.
standard_motion.send_trajectory()
