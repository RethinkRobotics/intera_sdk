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
from intera_interface import Limb

def main():
    """
    Move the robot arm to the specified configuration.
    Call using:
    $ rosrun intera_examples go_to_joint_angles.py  [arguments: see below]

    -q 0.0 0.0 0.0 0.0 0.0 0.0 0.0
    --> Go to joint pose: 0.0 0.0 0.0 0.0 0.0 0.0 0.0 using default settings

    -q 0.1 -0.2 0.15 -0.05 -0.08 0.14 -0.04 -s 0.1
    --> Go to pose [...] with a speed ratio of 0.1

    -q -0.2 0.1 0.1 0.2 -0.3 0.2 0.4 -s 0.9 -a 0.1
    --> Go to pose [...] with a speed ratio of 0.9 and an accel ratio of 0.1
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        "-q", "--joint_angles", type=float,
        nargs='+', default=[0.0, -0.9, 0.0, 1.8, 0.0, -0.9, 0.0],
        help="A list of joint angles, one for each of the 7 joints, J0...J6")
    parser.add_argument(
        "-s",  "--speed_ratio", type=float, default=0.5,
        help="A value between 0.001 (slow) and 1.0 (maximum joint velocity)")
    parser.add_argument(
        "-a",  "--accel_ratio", type=float, default=0.5,
        help="A value between 0.001 (slow) and 1.0 (maximum joint accel)")
    parser.add_argument(
        "--timeout", type=float, default=None,
        help="Max time in seconds to complete motion goal before returning. None is interpreted as an infinite timeout.")
    args = parser.parse_args(rospy.myargv()[1:])

    try:
        rospy.init_node('go_to_joint_angles_py')
        limb = Limb()
        traj = MotionTrajectory(limb = limb)

        wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=args.speed_ratio,
                                         max_joint_accel=args.accel_ratio)
        waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)

        joint_angles = limb.joint_ordered_angles()

        waypoint.set_joint_angles(joint_angles = joint_angles)
        traj.append_waypoint(waypoint.to_msg())

        if len(args.joint_angles) != len(joint_angles):
            rospy.logerr('The number of joint_angles must be %d', len(joint_angles))
            return None

        waypoint.set_joint_angles(joint_angles = args.joint_angles)
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
