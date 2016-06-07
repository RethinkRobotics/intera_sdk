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

import cv2
import cv_bridge

from sensor_msgs.msg import (
    Image,
)


def send_image(path):
    """
    Send the image located at the specified path to the head
    display on Baxter.

    @param path: path to the image file to load and send
    """
    img = cv2.imread(path)
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
    pub.publish(msg)
    # Sleep to allow for image to be published.
    rospy.sleep(1)


def main():
    """RSDK Xdisplay Example: Image Display

    Displays a given image file on Baxter's face.

    Pass the relative or absolute file path to an image file on your
    computer, and the example will read and convert the image using
    cv_bridge, sending it to the screen as a standard ROS Image Message.
    """
    epilog = """
Notes:
    Max screen resolution is 1024x600.
    Images are always aligned to the top-left corner.
    Image formats are those supported by OpenCv - LoadImage().
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-f', '--file', metavar='PATH', required=True,
        help='Path to image file to send'
    )
    parser.add_argument(
        '-d', '--delay', metavar='SEC', type=float, default=0.0,
        help='Time in seconds to wait before publishing image'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('rsdk_xdisplay_image', anonymous=True)

    if not os.access(args.file, os.R_OK):
        rospy.logerr("Cannot read file at '%s'" % (args.file,))
        return 1

    # Wait for specified time
    if args.delay > 0:
        rospy.loginfo(
            "Waiting for %s second(s) before publishing image to face" %
            (args.delay,)
        )
        rospy.sleep(args.delay)

    send_image(args.file)
    return 0

if __name__ == '__main__':
    sys.exit(main())
