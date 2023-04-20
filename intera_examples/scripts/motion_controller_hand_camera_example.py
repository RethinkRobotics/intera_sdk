#! /usr/bin/env python
#  Copyright (c) 2023, Rethink Robotics GmbH.
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

import rospy
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)

from intera_motion_msgs.msg import TrajectoryOptions
from intera_interface import (
    Limb,
    TipSelector
)

from geometry_msgs.msg import (
    PoseStamped
)
import numpy as np


def position_from_pose(pose):
    return np.array([pose.position.x, pose.position.y, pose.position.z])


rospy.init_node('montion_controller_hand_camera_example')
endpoint = 'right_hand_camera'

limb = Limb()

tips = TipSelector()
tips.add_camera_tip()
rospy.sleep(rospy.Duration(0.5))

wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=0.2,
                                 max_joint_accel=0.2)

wpt_opts.set_joint_tolerances(0.01)
# In this case, we would like to change the pose of the hand camera by 90°
# about the x-dir.


# Get the current state (pose, twist, wrench) of the right_hand_camera
state_camera = limb.tip_state(endpoint)
camera_pose = state_camera.pose

pose_new = PoseStamped()
pose_new.pose.position.x = 0.631
pose_new.pose.position.y = 0.1018
pose_new.pose.position.z = 0.6377
pose_new.pose.orientation.x = 0.4540262222290039
pose_new.pose.orientation.y = -0.5411508083343506
pose_new.pose.orientation.z = -0.47681906819343567
pose_new.pose.orientation.w = 0.5231248140335083

# Create a new trajectory
traj_opts = TrajectoryOptions()
traj_opts.interpolation_type = 'JOINT'  # Or 'CARTESIAN'
traj_opts.tracking_options.goal_joint_tolerance = [0.01] * 7
traj = MotionTrajectory(trajectory_options=traj_opts, limb=limb)

waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=limb,
                          active_endpoint=endpoint)

# Set the pose of the waypoint to newly create pose.
# Make sure to set the endpoint otherwise this will be overwritten.
# In addition, with TrajectoryOptions.interpolation_type = 'JOINT' the joint angles will be used
# instead of the Pose, hence call the inverse kinematics for the hand_camera and the Pose, and
# provide the resulting angles to the MotionWaypoint.set_cartesian_pose method.
ik_res = limb.ik_request(pose_new.pose, end_point=endpoint)
waypoint.set_cartesian_pose(pose_new, active_endpoint=endpoint, joint_angles=list(ik_res.values()))

# Set the waypoint options
traj.append_waypoint(waypoint)
# Execute the trajectory
traj.send_trajectory(timeout=5)
rospy.sleep(rospy.Duration(2))
# Get the current endpoint state (Pose, Twist, Wrench)
end_state = limb.tip_state(endpoint)
# print(position_from_pose(camera_pose) - position_from_pose(end_state.pose))
print('Distance to the goal: {0:.4f} m'.format(np.linalg.norm(
    position_from_pose(end_state.pose) - position_from_pose(pose_new.pose))))

# Create a CARTESIAN trajectory
traj_opts.interpolation_type = 'CARTESIAN'
traj = MotionTrajectory(trajectory_options=traj_opts, limb=limb)

pose_new.pose.position.x = 0.6634320020675659
pose_new.pose.position.y = 0.1334041804075241
pose_new.pose.position.z = 0.37226197123527527
pose_new.pose.orientation.x = -0.6865440607070923
pose_new.pose.orientation.y = 0.7257626056671143
pose_new.pose.orientation.z = -0.04118580371141434
pose_new.pose.orientation.w = -0.015151414088904858

waypoint.set_cartesian_pose(pose_new, active_endpoint=endpoint)
traj.append_waypoint(waypoint)
traj.send_trajectory(timeout=5)
rospy.sleep(rospy.Duration(2))
# Get the current endpoint state (Pose, Twist, Wrench)
end_state = limb.tip_state(endpoint)
print('Distance to the goal: {0:.4f} m'.format(np.linalg.norm(
    position_from_pose(end_state.pose) - position_from_pose(pose_new.pose))))

# Move hand camera back to the start

pose_new.pose.position.x = 0.631
pose_new.pose.position.y = 0.1018
pose_new.pose.position.z = 0.6377
pose_new.pose.orientation.x = 0.4540262222290039
pose_new.pose.orientation.y = -0.5411508083343506
pose_new.pose.orientation.z = -0.47681906819343567
pose_new.pose.orientation.w = 0.5231248140335083
ik_res = limb.ik_request(pose_new.pose, end_point=endpoint)
waypoint.set_cartesian_pose(pose_new, active_endpoint=endpoint,  joint_angles=list(ik_res.values()))
traj.append_waypoint(waypoint)

traj.send_trajectory(timeout=5)
rospy.sleep(rospy.Duration(2))
end_state = limb.tip_state(endpoint)
print('Distance to the goal: {0:.4f} m'.format(np.linalg.norm(
    position_from_pose(end_state.pose) - position_from_pose(pose_new.pose))))
