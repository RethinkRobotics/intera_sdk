#!/usr/bin/python2

# Copyright (c) 2013-2015, Rethink Robotics
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

import os
import sys
import argparse

import rospy
import xacro_jade

from baxter_core_msgs.msg import (
    URDFConfiguration,
)

def xacro_parse(filename):
    doc = xacro_jade.parse(None, filename)
    xacro_jade.process_doc(doc, in_order=True)
    return doc.toprettyxml(indent='  ')

def send_urdf(parent_link, root_joint, urdf_filename):
    """
    Send the URDF Fragment located at the specified path to
    modify the electric gripper on Baxter.

    @param parent_link: parent link to attach the URDF fragment to
                        (usually <side>_hand)
    @param root_joint: root link of the URDF fragment (usually <side>_gripper_base)
    @param urdf_filename: path to the urdf XML file to load into xacro and send
    """
    msg = URDFConfiguration()
    # The updating the time parameter tells
    # the robot that this is a new configuration.
    # Only update the time when an updated internal
    # model is required. Do not continuously update
    # the time parameter.
    msg.time = rospy.Time.now()
    # link to attach this urdf to onboard the robot
    msg.link = parent_link
    # root linkage in your URDF Fragment
    msg.joint = root_joint
    msg.urdf = xacro_parse(urdf_filename)
    pub = rospy.Publisher('/robot/urdf', URDFConfiguration, queue_size=10)
    rate = rospy.Rate(5) # 5hz
    while not rospy.is_shutdown():
        # Only one publish is necessary, but here we
        # will continue to publish until ctrl+c is invoked
        pub.publish(msg)
        rate.sleep()

def main():
    """RSDK URDF Fragment Example:
    This example shows a proof of concept for
    adding your URDF fragment to the robot's
    onboard URDF (which is currently in use).
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-f', '--file', metavar='PATH', required=True,
        help='Path to URDF file to send'
    )
    required.add_argument(
        '-l', '--link', required=False, default="left_hand",
        help='URDF Link already to attach fragment to (usually <left/right>_hand)'
    )
    required.add_argument(
        '-j', '--joint', required=False, default="left_gripper_base",
        help='Root joint for fragment (usually <left/right>_gripper_base)'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('rsdk_configure_urdf', anonymous=True)

    if not os.access(args.file, os.R_OK):
        rospy.logerr("Cannot read file at '%s'" % (args.file,))
        return 1
    send_urdf(args.link, args.joint, args.file)
    return 0

if __name__ == '__main__':
    sys.exit(main())
