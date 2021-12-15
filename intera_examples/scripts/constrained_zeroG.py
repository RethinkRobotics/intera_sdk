#! /usr/bin/env python
# Copyright (c) 2018, Rethink Robotics Inc.
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
from intera_core_msgs.msg import InteractionControlCommand
from geometry_msgs.msg import Pose
from intera_motion_interface import (InteractionOptions, InteractionPublisher)
from intera_motion_interface.utility_functions import (int2bool, bool2int, boolToggle)

def main():
    """
    Initiate constrained zero-G with a desired behavior from the current pose.
    The desired behavior can be specified by the optional arguments. By default,
    all directions will be set unconstrained when no options are provided.
    Note that the arm is not commanded to move but it will have the specified
    interaction control behavior.

    If publish the rate is 0, the interaction control command is only published
    once; otherwise a last position command will be sent when the script exits.
    For non-zero publishing rates, the arm will go back into constrained zero-G
    if the arm's zero-g button is pressed and relased. Future motion commands
    will use the interaction parameters set in the trajectory options.

    Call using:
    $ rosrun intera_examples constrained_zeroG.py  [arguments: see below]

    -p
    --> Allow for arbitrary endpoint position with fixed orientation

    -o
    --> Allow for arbitrary endpoint orientation with fixed position

    -ph
    --> Allow for arbitrary endpoint position on a horizontal plane (XY plane) with fixed orientation

    -yz
    --> Allow for arbitrary endpoint position on a vertical YZ plane with fixed orientation

    -px
    --> Allow for arbitrary endpoint position along x-axis

    -px -oy
    --> Allow for arbitrary endpoint position along x-axis as well as arbitrary endpoint orientation about y-axis

    -ca 1 1 0 1 1 1
    --> Allow for arbitrary translational movement along z-axis only

    -ca 1 1 1 0 1 1
    --> Allow for arbitrary rotational movement about x-axis only

    -fr 0.1 0.2 0.3 1 0 0 0
    --> Set the pose of the interaction_frame -- position: (0.1, 0.2, 0.3) and orientation (1, 0, 0, 0)

    -ef
    --> Set in_endpoint_frame to True in the current configuration (use TCP frame as reference frame)

    -kn 5.0 3.0 5.0 4.0 6.0 4.0 6.0
    --> Set K_nullspace to [5.0 3.0 5.0 4.0 6.0 4.0 6.0] in the current configuration

    -kn 0.0
    --> Set K_nullspace to [0 0 0 0 0 0 0] in the current configuration

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
        "-p",  "--position_only", action='store_true', default=False,
        help="Allow for arbitrary endpoint position with fixed orientation")
    parser.add_argument(
        "-o",  "--orientation_only", action='store_true', default=False,
        help="Allow for arbitrary endpoint orientation with fixed position")
    parser.add_argument(
        "-ph",  "--plane_horizontal", action='store_true', default=False,
        help="Allow for arbitrary endpoint position on a horizontal plane (XY plane) with fixed orientation")
    parser.add_argument(
        "-xz",  "--plane_vertical_xz", action='store_true', default=False,
        help="Allow for arbitrary endpoint position on a vertical plane (XZ plane) with fixed orientation")
    parser.add_argument(
        "-yz",  "--plane_vertical_yz", action='store_true', default=False,
        help="Allow for arbitrary endpoint position on a vertical plane (YZ plane) with fixed orientation")
    parser.add_argument(
        "-ns",  "--nullspace_only", action='store_true', default=False,
        help="Allow for arbitrary movement only in the nullspace of the configuration with fixed endpoint pose")
    parser.add_argument(
        "-px",  "--position_x", action='store_true', default=False,
        help="Allow for arbitrary endpoint position along x-axis")
    parser.add_argument(
        "-py",  "--position_y", action='store_true', default=False,
        help="Allow for arbitrary endpoint position along y-axis")
    parser.add_argument(
        "-pz",  "--position_z", action='store_true', default=False,
        help="Allow for arbitrary endpoint position along z-axis")
    parser.add_argument(
        "-ox",  "--orientation_x", action='store_true', default=False,
        help="Allow for arbitrary endpoint orientation about x-axis")
    parser.add_argument(
        "-oy",  "--orientation_y", action='store_true', default=False,
        help="Allow for arbitrary endpoint orientation about y-axis")
    parser.add_argument(
        "-oz",  "--orientation_z", action='store_true', default=False,
        help="Allow for arbitrary endpoint orientation about z-axis")
    parser.add_argument(
        "-ca", "--constrained_axes", type=int,
        nargs=6, default=[1, 1, 1, 1, 1, 1], choices = [0, 1],
        help="A list of Cartesian axes with maximum stiffness, 0 (zero stiffness) or 1 (maximum stiffness) for each of the 6 directions (3 translational directions followed by 3 rotational directions)")
    parser.add_argument(
        "-ef",  "--in_endpoint_frame", action='store_true', default=False,
        help="Set the desired reference frame to endpoint frame ('right_hand'); otherwise, it is base frame by default")
    parser.add_argument(
        "-fr", "--interaction_frame", type=float,
        nargs='+', default=[0, 0, 0, 1, 0, 0, 0],
        help="Specify the reference frame for the interaction controller -- first 3 values are positions [m] and last 4 values are orientation in quaternion (w, x, y, z) which has to be normalized values")
    parser.add_argument(
        "-kn", "--K_nullspace", type=float,
        nargs='+', default=[10.0, 10.0, 7.0, 0.0, 0.0, 0.0, 0.0],
        help="A list of desired nullspace stiffnesses, one for each of the 7 joints (a single value can be provided to apply the same value to all the directions) -- units are in (Nm/rad)")
    parser.add_argument(
        "-r",  "--rate", type=int, default=10,
        help="A desired publish rate for updating interaction control commands (10Hz by default) -- a rate 0 publish once and exits which can cause the arm to remain in interaction control.")

    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node('constrained_zeroG')

    # set the interaction control options in the current configuration
    interaction_options = InteractionOptions()

    # if one of the options is set
    unconstrained_axes_default = [0, 0, 0, 0, 0, 0]
    unconstrained_axes = unconstrained_axes_default

    # create a list of the constrained zero G modes
    sum_mode_list = sum(bool2int([args.position_only, args.orientation_only,
                                  args.plane_horizontal, args.plane_vertical_xz,
                                  args.plane_vertical_yz, args.nullspace_only]))

    # create a list of the free axis options
    free_axis_list = bool2int([args.position_x, args.position_y, args.position_z,
                               args.orientation_x, args.orientation_y, args.orientation_z])
    sum_free_axis_list = sum(free_axis_list)

    zero_stiffness_axes = boolToggle(args.constrained_axes)
    sum_zero_stiffness_axes = sum(zero_stiffness_axes)

    if (sum_mode_list==0 and sum_zero_stiffness_axes==0 and sum_free_axis_list==0):
        rospy.logerr('You need to either set one of the options or specify the ' \
                     'desired axes of arbitrary movement!')
        rospy.logerr('The movement of endpoint will be constrained in all directions!')

    if (sum_mode_list>1 and sum_zero_stiffness_axes==0 and sum_free_axis_list==0):
        rospy.logerr('You can set only one of the options among "pos_only", "ori_only", ' \
                     '"plane_hor", and "plane_ver"!')
        rospy.logerr('The movement of endpoint will be constrained in all directions!')

    # give an error if the mode options as well as the individual axis options are set
    if (sum_mode_list==1 and sum_zero_stiffness_axes==0 and sum_free_axis_list>0):
        rospy.logerr('The individual axis options cannot be used together with the mode options!')
        rospy.logerr('The movement of endpoint will be constrained in all directions!')

    # give an error when the axes are specified by more than one method
    if (sum_mode_list==0 and sum_zero_stiffness_axes>0 and sum_free_axis_list>0):
        rospy.logerr('You can only set the axes either by an array or individual options!')
        rospy.logerr('The movement of endpoint will be constrained in all directions!')

    if (sum_mode_list==1 and sum_zero_stiffness_axes==0 and sum_free_axis_list>=0):
        if args.position_only:
            unconstrained_axes = [1, 1, 1, 0, 0, 0]
        if args.orientation_only:
            unconstrained_axes = [0, 0, 0, 1, 1, 1]
        if args.plane_horizontal:
            unconstrained_axes = [1, 1, 0, 0, 0, 0]
        if args.plane_vertical_xz:
            unconstrained_axes = [1, 0, 1, 0, 0, 0]
        if args.plane_vertical_yz:
            unconstrained_axes = [0, 1, 1, 0, 0, 0]
        if args.nullspace_only:
            unconstrained_axes = [0, 0, 0, 0, 0, 0]

    # if the axes are specified by an array
    if (sum_mode_list==0 and sum_zero_stiffness_axes>0 and sum_free_axis_list==0):
        unconstrained_axes = zero_stiffness_axes

    # if the axes are specified by individual options
    if (sum_mode_list==0 and sum_zero_stiffness_axes==0 and sum_free_axis_list>0):
        unconstrained_axes = free_axis_list

    # set the stiffness to zero by default
    interaction_options.set_K_impedance([0, 0, 0, 0, 0, 0])

    # set the axes with maximum stiffness
    interaction_options.set_max_impedance(boolToggle(int2bool(unconstrained_axes)))

    interaction_options.set_in_endpoint_frame(args.in_endpoint_frame)

    # set nullspace stiffness to zero if nullspace_only option is provided
    if args.nullspace_only:
        K_nullspace = [0, 0, 0, 0, 0, 0, 0]
    else:
        K_nullspace = args.K_nullspace

    interaction_options.set_K_nullspace(K_nullspace)

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

    # always enable the rotations for constrained zero-G
    interaction_options.set_rotations_for_constrained_zeroG(True)

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
