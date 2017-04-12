#! /usr/bin/env python

# Copyright (c) 2016-2017, Rethink Robotics Inc.
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
from intera_core_msgs.msg import InteractionControlCommand
import numpy as np
import argparse
from copy import deepcopy
from geometry_msgs.msg import Pose
from motion_interface import InteractionOptions

def int2bool(var):
    """
    Convert integer value/list to bool value/list
    """
    var_out = deepcopy(var)

    if isinstance(var, int):
        var_out = bool(var)
    elif len(var) >= 1:
        for i in range(0, len(var)):
            var_out[i] = bool(var[i])

    return var_out

def main():
    """
    Set the desired interaction control options in the current configuration.
    Note that the arm is not commanded to move but it will have the specified
    interaction control behavior. If publish rate is 0 where the interaction
    control command is only published once, after entering and exiting zero-G,
    the arm will return to normal position mode. Also, regardless of the publish
    rate, the zero-G behavior will not be affected by this. The future motion
    commands need to be sent with interaction parameters if we want to keep
    interaction control behaviors during the trajectory execution; otherwise,
    the arm will move in position mode during the motion even if this script
    is still running.

    Call using:
    $ rosrun motion_interface set_interaction_options.py  [arguments: see below]

    -s 1
    --> Set interaction active to True (0 for False) in the current configuration

    -k 500.0 500.0 500.0 10.0 10.0 10.0
    --> Set K_impedance to [500.0 500.0 500.0 10.0 10.0 10.0] in the current configuration

    -m 1 1 0 1 1 1
    --> Set max_impedance to [True True False True True True] in the current configuration

    -md 1 1 2 1 1 1
    --> Set interaction_control_mode to [impedance, impedance, force, impedance, impedance, impedance]
        in the current configuration (1: impedance, 2: force, 3: impedance w/ force limit,
        4: force w/ motion limit)

    -fr 0.1 0.2 0.3 1 0 0 0
    --> Set the pose of the interaction_frame -- position: (0.1, 0.2, 0.3) and orientation (1, 0, 0, 0)

    -ef
    --> Set in_endpoint_frame to True in the current configuration

    -en 'right_hand'
    --> Specify the desired endpoint frame where impedance and force control behaviors are defined

    -f 0.0 0.0 30.0 0.0 0.0 0.0
    --> Set force_command to [0.0 0.0 30.0 0.0 0.0 0.0] in the current configuration

    -kn 5.0 3.0 5.0 4.0 6.0 4.0 6.0
    --> Set K_nullspace to [5.0 3.0 5.0 4.0 6.0 4.0 6.0] in the current configuration

    -r 20
    --> Set desired publish rate (Hz)
    """

    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        "-s",  "--interaction_active", type=int, default=1, choices = [0, 1],
        help="Activate (1) or Deactivate (0) interaction controller")
    parser.add_argument(
        "-k", "--K_impedance", type=float,
        nargs='+', default=[1300.0, 1300.0, 1300.0, 30.0, 30.0, 30.0],
        help="A list of desired stiffnesses, one for each of the 6 directions -- stiffness units are (N/m) for first 3 and (Nm/rad) for second 3 values")
    parser.add_argument(
        "-m", "--max_impedance", type=int,
        nargs='+', default=[1, 1, 1, 1, 1, 1], choices = [0, 1],
        help="A list of maximum stiffness behavior state, one for each of the 6 directions (a single value can be provided to apply the same value to all the directions) -- 0 for False, 1 for True")
    parser.add_argument(
        "-md", "--interaction_control_mode", type=int,
        nargs='+', default=[1, 1, 1, 1, 1, 1], choices = [1,2,3,4],
        help="A list of desired interaction control mode (1: impedance, 2: force, 3: impedance with force limit, 4: force with motion limit), one for each of the 6 directions")
    parser.add_argument(
        "-fr", "--interaction_frame", type=float,
        nargs='+', default=[0, 0, 0, 1, 0, 0, 0],
        help="Specify the reference frame for the interaction controller -- first 3 values are positions [m] and last 4 values are orientation in quaternion (w, x, y, z) which has to be normalized values")
    parser.add_argument(
        "-ef",  "--in_endpoint_frame", action='store_true', default=False,
        help="Set the desired reference frame to endpoint frame; otherwise, it is base frame by default")
    parser.add_argument(
        "-en",  "--endpoint_name", type=str, default='right_hand',
        help="Set the desired endpoint frame by its name; otherwise, it is right_hand frame by default")
    parser.add_argument(
        "-f", "--force_command", type=float,
        nargs='+', default=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        help="A list of desired force commands, one for each of the 6 directions -- in force control mode this is the vector of desired forces/torques to be regulated in (N) and (Nm), in impedance with force limit mode this vector specifies the magnitude of forces/torques (N and Nm) that the command will not exceed")
    parser.add_argument(
        "-kn", "--K_nullspace", type=float,
        nargs='+', default=[4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0],
        help="A list of desired nullspace stiffnesses, one for each of the 7 joints (a single value can be provided to apply the same value to all the directions) -- units are in (Nm/rad)")
    parser.add_argument(
        "-r",  "--rate", type=int, default=10,
        help="A desired publish rate for updating interaction control commands (10Hz by default) -- 0 if we want to publish it only once")

    args = parser.parse_args()

    try:
        rospy.init_node('set_interaction_options_py')
        pub = rospy.Publisher('/robot/limb/right/interaction_control_command', InteractionControlCommand, queue_size=10)
        rospy.sleep(0.5)

        if args.rate > 0:
            rate = rospy.Rate(args.rate) # 10hz
        elif args.rate == 0:
            rospy.logwarn('Interaction control options will be set only once!')
        elif args.rate < 0:
            rospy.logerr('Invalid publish rate!')

        # set the interaction control options in the current configuration
        interaction_options = InteractionOptions()

        if args.interaction_active is not None:
            interaction_options.set_interaction_control_active(int2bool(args.interaction_active))
        if args.K_impedance is not None:
            interaction_options.set_K_impedance(args.K_impedance)
        if args.max_impedance is not None:
            interaction_options.set_max_impedance(int2bool(args.max_impedance))
        if args.interaction_control_mode is not None:
            interaction_options.set_interaction_control_mode(args.interaction_control_mode)
        if args.in_endpoint_frame is not None:
            interaction_options.set_in_endpoint_frame(int2bool(args.in_endpoint_frame))
        if args.force_command is not None:
            interaction_options.set_force_command(args.force_command)
        if args.K_nullspace is not None:
            interaction_options.set_K_nullspace(args.K_nullspace)
        if args.endpoint_name is not None:
            interaction_options.set_endpoint_name(args.endpoint_name)
        if args.interaction_frame is not None:
            if len(args.interaction_frame) < 7:
                rospy.logerr('The number of elements must be 7!')
            elif len(args.interaction_frame) == 7:
                quat = np.array([args.interaction_frame[3], args.interaction_frame[4], args.interaction_frame[5], args.interaction_frame[6]])
                if np.linalg.norm(quat) < 1.0 + 1e-7 and np.linalg.norm(quat) > 1.0 - 1e-7:
                    interaction_frame = Pose()
                    interaction_frame.position.x = args.interaction_frame[0]
                    interaction_frame.position.y = args.interaction_frame[1]
                    interaction_frame.position.z = args.interaction_frame[2]
                    interaction_frame.orientation.w = args.interaction_frame[3]
                    interaction_frame.orientation.x = args.interaction_frame[4]
                    interaction_frame.orientation.y = args.interaction_frame[5]
                    interaction_frame.orientation.z = args.interaction_frame[6]
                    interaction_options.set_interaction_frame(interaction_frame)
                else:
                    rospy.logerr('Invalid input to quaternion!')
            else:
                rospy.logerr('Invalid input to interaction_frame!')

        msg = interaction_options.to_msg()

        if args.rate > 0:
            while not rospy.is_shutdown():
                # print the resultant interaction options
                rospy.loginfo(msg)
                pub.publish(msg)
                rate.sleep()
        else:
            # print the resultant interaction options
            rospy.loginfo(msg)
            pub.publish(msg)


    except rospy.ROSInterruptException:
        rospy.logerr('Keyboard interrupt detected from the user. %s',
                     'Exiting the node...')


if __name__ == '__main__':
    main()
