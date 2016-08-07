#!/usr/bin/env python

# Copyright (c) 2013-2016, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Intera SDK Joint Trajectory Controller
    Unlike other robots running ROS, this is not a Motor Controller plugin,
    but a regular node using the SDK interface.
"""
import argparse
import importlib

import rospy
from dynamic_reconfigure.server import Server
import intera_interface

from joint_trajectory_action.joint_trajectory_action import (
    JointTrajectoryActionServer,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

def start_server(limb, rate, mode, valid_limbs):
    rospy.loginfo("Initializing node... ")
    rospy.init_node("sdk_{0}_joint_trajectory_action_server{1}".format(
                        mode, "" if limb == 'all_limbs' else "_" + limb,))

    rospy.loginfo("Initializing joint trajectory action server...")
    robot_name = intera_interface.RobotParams().get_robot_name().lower().capitalize()
    config_module = "intera_interface.cfg"
    if mode == 'velocity':
        config_name = ''.join([robot_name,"VelocityJointTrajectoryActionServerConfig"])
    elif mode == 'position':
        config_name = ''.join([robot_name,"PositionJointTrajectoryActionServerConfig"])
    else:
        config_name = ''.join([robot_name,"PositionFFJointTrajectoryActionServerConfig"])
    cfg = importlib.import_module('.'.join([config_module,config_name]))
    dyn_cfg_srv = Server(cfg, lambda config, level: config)
    jtas = []
    if limb == 'all_limbs':
        for current_limb in valid_limbs:
            jtas.append(JointTrajectoryActionServer(current_limb, dyn_cfg_srv,
                                                    rate, mode))
    else:
        jtas.append(JointTrajectoryActionServer(limb, dyn_cfg_srv, rate, mode))


    def cleanup():
        for j in jtas:
            j.clean_shutdown()

    rospy.on_shutdown(cleanup)
    rospy.loginfo("Joint Trajectory Action Server Running. Ctrl-c to quit")
    rospy.spin()


def main():
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message(("Cannot detect any limb parameters on this robot. "
                        "Exiting."), "ERROR")
        return
    # Add an option for starting a server for all valid limbs
    all_limbs = valid_limbs
    all_limbs.append("all_limbs")
    arg_fmt = argparse.ArgumentDefaultsHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt)
    parser.add_argument(
        "-l", "--limb", dest="limb", default=valid_limbs[0],
        choices=all_limbs,
        help="joint trajectory action server limb"
    )
    parser.add_argument(
        "-r", "--rate", dest="rate", default=100.0,
        type=float, help="trajectory control rate (Hz)"
    )
    parser.add_argument(
        "-m", "--mode", default='position_w_id',
        choices=['position_w_id', 'position', 'velocity'],
        help="control mode for trajectory execution"
    )
    args = parser.parse_args(rospy.myargv()[1:])
    start_server(args.limb, args.rate, args.mode, valid_limbs)


if __name__ == "__main__":
    main()
