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

import sys
import rospy
import argparse
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions,
    RandomWalk
)
from intera_motion_msgs.msg import TrajectoryOptions
from intera_core_msgs.msg import JointLimits
from intera_interface import (
    Limb,
    JointLimits
)

if (sys.version_info < (3, 0)):
  input = raw_input

def main():
    """
    Send a random-walk trajectory, starting from the current pose of the robot.
    Can be used to send both joint and cartesian trajectories.

    WARNING: Make sure the surrounding area around the robot is collision free
             prior to sending random trajectories.

    Call using:
    $ rosrun intera_examples send_random_trajectory.py  [arguments: see below]

    -n 5 -t JOINT -s 0.5
    --> Send a trandom joint trajectory with 5 waypoints, using a speed ratio
        of 0.5 for all waypoints. Use default random walk settings.

    -d 0.1 -b 0.2
    --> Send a random trajectory with default trajectory settings. Use a
        maximum step distance of 0.1*(upper joint limit - lower joint limit)
        and avoid the upper and lower joint limits by a boundary of
        0.2*(upper joint limit - lower joint limit).

    -o ~/Desktop/fileName.bag
    --> Save the trajectory message to a bag file

    --seed 1234
    --> Set the seed in the random number generator before constructing the
        trajectory. This allows for repeatable motion and is useful for
        writing repeatable tests that use this script. Note: for a repeatable
        test the starting pose of the robot must also be the same each time.

    -p
    --> Prints the trajectory to terminal before sending

    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        "-n", "--waypoint_count", type=int, default=5,
        help="number of waypoints to include in the trajectory")
    parser.add_argument(
        "-d", "--stepDistance", type=float, default=0.05,
        help="normalized random walk step distance")
    parser.add_argument(
        "-b", "--boundaryPadding", type=float, default=0.1,
        help="normalized padding to apply to joint limits")
    parser.add_argument(
        "-t", "--trajType", type=str, default='JOINT',
        choices=['JOINT', 'CARTESIAN'],
        help="trajectory interpolation type")
    parser.add_argument(
        "-s",  "--speed_ratio", type=float, default=0.5,
        help="A value between 0.0 (slow) and 1.0 (fast)")
    parser.add_argument(
        "-a",  "--accel_ratio", type=float, default=0.5,
        help="A value between 0.0 (slow) and 1.0 (fast)")
    parser.add_argument(
        "-o", "--output_file",
        help="Save the trajectory task to a bag file")
    parser.add_argument(
        "--output_file_type", default="yaml",
        choices=["csv", "yaml"], help="Select the save file type")
    parser.add_argument(
        "-p", "--print_trajectory", action='store_true', default=False,
        help="print the trajectory after loading")
    parser.add_argument(
        "--do_not_send", action='store_true', default=False,
        help="generate the trajectory, but do not send to motion controller.")
    parser.add_argument(
        "--log_file_output",
        help="Save motion controller log messages to this file name")
    parser.add_argument(
        "--timeout", type=float, default=None,
        help="Max time in seconds to complete motion goal before returning. None is interpreted as an infinite timeout.")
    parser.add_argument(
        "--rng_seed", type=int, default=0,
        help="specify a seed to pass to the random number generator. Set to zero to use default initialization.")
    args = parser.parse_args()

    if args.waypoint_count < 1:
        args.waypoint_count = 1
        rospy.logwarn('Input out of bounds! Setting waypoint_count = 1')

    try:
        rospy.init_node('send_random_joint_trajectory_py')

        if not args.do_not_send:
            rospy.logwarn('Make sure the surrounding area around the robot is '
                          'collision free prior to sending random trajectories.')
            k = input("Press 'Enter' when the robot is clear to continue...")
            if k:
                rospy.logerr("Please press only the 'Enter' key to begin execution. Exiting...")
                sys.exit(1)

        # Set the trajectory options
        limb = Limb()
        traj_opts = TrajectoryOptions()
        traj_opts.interpolation_type = args.trajType
        traj = MotionTrajectory(trajectory_options = traj_opts, limb = limb)

        # Set the waypoint options
        wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=args.speed_ratio,
                                        max_joint_accel=args.accel_ratio)
        waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)

        # Append a waypoint at the current pose
        waypoint.set_joint_angles(limb.joint_ordered_angles())
        traj.append_waypoint(waypoint.to_msg())

        # Set up the random walk generator
        walk = RandomWalk(seed=args.rng_seed)
        limits = JointLimits()
        walk.set_lower_limits(limits.get_joint_lower_limits(limb.joint_names()))
        walk.set_upper_limits(limits.get_joint_upper_limits(limb.joint_names()))
        walk.set_last_point(waypoint.get_joint_angles())
        walk.set_boundary_padding(args.boundaryPadding)
        walk.set_maximum_distance(args.stepDistance)

        for i in range(0, args.waypoint_count):
            joint_angles = walk.get_next_point()
            waypoint.set_joint_angles(joint_angles = joint_angles)
            traj.append_waypoint(waypoint.to_msg())

        if args.output_file is not None:
            if args.output_file_type == "csv":
                traj.to_csv_file(args.output_file)
            elif args.output_file_type == "yaml":
                traj.to_yaml_file(args.output_file)
            else:
                rospy.logwarn("Did not recognize output file type")

        if args.print_trajectory:
            rospy.loginfo('\n' + traj.to_string())

        if args.log_file_output is not None:
            traj.set_log_file_name(args.log_file_output)

        if not args.do_not_send:
            result = traj.send_trajectory(timeout=args.timeout)
            if result is None:
                rospy.logerr('Trajectory FAILED to send')
            elif result.result:
                rospy.loginfo('Motion controller successfully finished the trajectory!')
            else:
                rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                             result.errorId)
    except rospy.ROSInterruptException:
        rospy.logerr('Keyboard interrupt detected from the user. '
                     'Exiting before trajectory completion.')

if __name__ == '__main__':
    main()
