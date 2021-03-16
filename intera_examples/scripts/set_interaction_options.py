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
from geometry_msgs.msg import Pose
from intera_core_msgs.msg import InteractionControlCommand
from intera_motion_interface import (InteractionOptions, InteractionPublisher)
from intera_motion_interface.utility_functions import int2bool

def main():
    """
    Set the desired interaction control options in the current configuration.
    Note that the arm is not commanded to move but it will have the specified
    interaction control behavior.

    If publish the rate is 0, the interaction control command is only published
    once; otherwise a last position command will be sent when the script exits.
    For non-zero publishing rates, the arm will go back into constrained zero-G
    if the arm's zero-g button is pressed and relased. Future motion commands
    will use the interaction parameters set in the trajectory options.

    Call using:
    $ rosrun intera_examples set_interaction_options.py  [arguments: see below]

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
    --> Set in_endpoint_frame to True in the current configuration (use TCP frame as reference frame)

    -f 0.0 0.0 30.0 0.0 0.0 0.0
    --> Set force_command to [0.0 0.0 30.0 0.0 0.0 0.0] in the current configuration

    -kn 5.0 3.0 5.0 4.0 6.0 4.0 6.0
    --> Set K_nullspace to [5.0 3.0 5.0 4.0 6.0 4.0 6.0] in the current configuration

    -r 20
    --> Set desired publish rate (Hz)

    -r 0
    --> The interaction command is published once and exits. The arm can remain
        in interaction control after this script.
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
        nargs='+', default=[5.0, 10.0, 5.0, 10.0, 5.0, 10.0, 5.0],
        help="A list of desired nullspace stiffnesses, one for each of the 7 joints (a single value can be provided to apply the same value to all the directions) -- units are in (Nm/rad)")
    parser.add_argument(
        "-dd",  "--disable_damping_in_force_control", action='store_true', default=False,
        help="Disable damping in force control")
    parser.add_argument(
        "-dr",  "--disable_reference_resetting", action='store_true', default=False,
        help="The reference signal is reset to actual position to avoid jerks/jumps when interaction parameters are changed. This option allows the user to disable this feature.")
    parser.add_argument(
        "-rc",  "--rotations_for_constrained_zeroG", action='store_true', default=False,
        help="Allow arbitrary rotational displacements from the current orientation for constrained zero-G (works only with a stationary reference orientation)")
    parser.add_argument(
        "-r",  "--rate", type=int, default=10,
        help="A desired publish rate for updating interaction control commands (10Hz by default) -- a rate 0 publish once and exits which can cause the arm to remain in interaction control.")

    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node('set_interaction_options')

    # set the interaction control options in the current configuration
    interaction_options = InteractionOptions()

    interaction_options.set_interaction_control_active(int2bool(args.interaction_active))
    interaction_options.set_K_impedance(args.K_impedance)
    interaction_options.set_max_impedance(int2bool(args.max_impedance))
    interaction_options.set_interaction_control_mode(args.interaction_control_mode)
    interaction_options.set_in_endpoint_frame(args.in_endpoint_frame)
    interaction_options.set_force_command(args.force_command)
    interaction_options.set_K_nullspace(args.K_nullspace)

    if len(args.interaction_frame) == 7:
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
        rospy.logerr('Invalid input to interaction_frame. Must be 7 elements.')

    interaction_options.set_disable_damping_in_force_control(args.disable_damping_in_force_control)
    interaction_options.set_disable_reference_resetting(args.disable_reference_resetting)
    interaction_options.set_rotations_for_constrained_zeroG(args.rotations_for_constrained_zeroG)

    # print the resultant interaction options once
    rospy.loginfo(interaction_options.to_msg())

    ic_pub = InteractionPublisher()
    rospy.sleep(0.5)
    if args.rate != 0:
        rospy.on_shutdown(ic_pub.send_position_mode_cmd)
    ic_pub.send_command(interaction_options, args.rate)
    if args.rate == 0:
        rospy.sleep(0.5)


if __name__ == '__main__':
    main()
