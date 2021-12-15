#! /usr/bin/env python
# Copyright (c) 2016-2018, Rethink Robotics Inc.
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
import argparse
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
from intera_motion_msgs.msg import TrajectoryOptions
from geometry_msgs.msg import PoseStamped
import PyKDL
from tf_conversions import posemath
from intera_interface import Limb

def main():
    """
    Move the robot arm to the specified configuration.
    Call using:
    $ rosrun intera_examples go_to_cartesian_pose.py  [arguments: see below]

    -p 0.4 -0.3 0.18 -o 0.0 1.0 0.0 0.0 -t right_hand
    --> Go to position: x=0.4, y=-0.3, z=0.18 meters
    --> with quaternion orientation (0, 1, 0, 0) and tip name right_hand
    --> The current position or orientation will be used if only one is provided.

    -q 0.0 -0.9 0.0 1.8 0.0 -0.9 0.0
    --> Go to joint angles: 0.0 -0.9 0.0 1.8 0.0 -0.9 0.0 using default settings
    --> If a Cartesian pose is not provided, Forward kinematics will be used
    --> If a Cartesian pose is provided, the joint angles will be used to bias the nullspace

    -R 0.01 0.02 0.03 0.1 0.2 0.3 -T
    -> Jog arm with Relative Pose (in tip frame)
    -> x=0.01, y=0.02, z=0.03 meters, roll=0.1, pitch=0.2, yaw=0.3 radians
    -> The fixed position and orientation paramters will be ignored if provided

    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        "-p", "--position", type=float,
        nargs='+',
        help="Desired end position: X, Y, Z")
    parser.add_argument(
        "-o", "--orientation", type=float,
        nargs='+',
        help="Orientation as a quaternion (x, y, z, w)")
    parser.add_argument(
        "-R", "--relative_pose", type=float,
        nargs='+',
        help="Jog pose by a relative amount in the base frame: X, Y, Z, roll, pitch, yaw")
    parser.add_argument(
        "-T", "--in_tip_frame", action='store_true',
        help="For relative jogs, job in tip frame (default is base frame)")
    parser.add_argument(
        "-q", "--joint_angles", type=float,
        nargs='+', default=[],
        help="A list of joint angles, one for each of the 7 joints, J0...J6")
    parser.add_argument(
        "-t",  "--tip_name", default='right_hand',
        help="The tip name used by the Cartesian pose")
    parser.add_argument(
        "--linear_speed", type=float, default=0.6,
        help="The max linear speed of the endpoint (m/s)")
    parser.add_argument(
        "--linear_accel", type=float, default=0.6,
        help="The max linear acceleration of the endpoint (m/s/s)")
    parser.add_argument(
        "--rotational_speed", type=float, default=1.57,
        help="The max rotational speed of the endpoint (rad/s)")
    parser.add_argument(
        "--rotational_accel", type=float, default=1.57,
        help="The max rotational acceleration of the endpoint (rad/s/s)")
    parser.add_argument(
        "--timeout", type=float, default=None,
        help="Max time in seconds to complete motion goal before returning. None is interpreted as an infinite timeout.")
    args = parser.parse_args(rospy.myargv()[1:])

    try:
        rospy.init_node('go_to_cartesian_pose_py')
        limb = Limb()

        traj_options = TrajectoryOptions()
        traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
        traj = MotionTrajectory(trajectory_options = traj_options, limb = limb)

        wpt_opts = MotionWaypointOptions(max_linear_speed=args.linear_speed,
                                         max_linear_accel=args.linear_accel,
                                         max_rotational_speed=args.rotational_speed,
                                         max_rotational_accel=args.rotational_accel,
                                         max_joint_speed_ratio=1.0)
        waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)

        joint_names = limb.joint_names()

        if args.joint_angles and len(args.joint_angles) != len(joint_names):
            rospy.logerr('len(joint_angles) does not match len(joint_names!)')
            return None

        if (args.position is None and args.orientation is None
            and args.relative_pose is None):
            if args.joint_angles:
                # does Forward Kinematics
                waypoint.set_joint_angles(args.joint_angles, args.tip_name, joint_names)
            else:
                rospy.loginfo("No Cartesian pose or joint angles given. Using default")
                waypoint.set_joint_angles(joint_angles=None, active_endpoint=args.tip_name)
        else:
            endpoint_state = limb.tip_state(args.tip_name)
            if endpoint_state is None:
                rospy.logerr('Endpoint state not found with tip name %s', args.tip_name)
                return None
            pose = endpoint_state.pose

            if args.relative_pose is not None:
                if len(args.relative_pose) != 6:
                    rospy.logerr('Relative pose needs to have 6 elements (x,y,z,roll,pitch,yaw)')
                    return None
                # create kdl frame from relative pose
                rot = PyKDL.Rotation.RPY(args.relative_pose[3],
                                         args.relative_pose[4],
                                         args.relative_pose[5])
                trans = PyKDL.Vector(args.relative_pose[0],
                                     args.relative_pose[1],
                                     args.relative_pose[2])
                f2 = PyKDL.Frame(rot, trans)
                # and convert the result back to a pose message
                if args.in_tip_frame:
                  # end effector frame
                  pose = posemath.toMsg(posemath.fromMsg(pose) * f2)
                else:
                  # base frame
                  pose = posemath.toMsg(f2 * posemath.fromMsg(pose))
            else:
                if args.position is not None and len(args.position) == 3:
                    pose.position.x = args.position[0]
                    pose.position.y = args.position[1]
                    pose.position.z = args.position[2]
                if args.orientation is not None and len(args.orientation) == 4:
                    pose.orientation.x = args.orientation[0]
                    pose.orientation.y = args.orientation[1]
                    pose.orientation.z = args.orientation[2]
                    pose.orientation.w = args.orientation[3]
            poseStamped = PoseStamped()
            poseStamped.pose = pose

            if not args.joint_angles:
                # using current joint angles for nullspace bais if not provided
                joint_angles = limb.joint_ordered_angles()
                waypoint.set_cartesian_pose(poseStamped, args.tip_name, joint_angles)
            else:
                waypoint.set_cartesian_pose(poseStamped, args.tip_name, args.joint_angles)



        rospy.loginfo('Sending waypoint: \n%s', waypoint.to_string())

        traj.append_waypoint(waypoint.to_msg())

        result = traj.send_trajectory(timeout=args.timeout)
        if result is None:
            rospy.logerr('Trajectory FAILED to send')
            return

        if result.result:
            rospy.loginfo('Motion controller successfully finished the trajectory!')
        else:
            rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                         result.errorId)
    except rospy.ROSInterruptException:
        rospy.logerr('Keyboard interrupt detected from the user. Exiting before trajectory completion.')


if __name__ == '__main__':
    main()
