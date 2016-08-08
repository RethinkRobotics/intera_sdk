#!/usr/bin/env python

# Copyright (c) 2016, Rethink Robotics
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
import cv2
import cv_bridge
import rospy

from sensor_msgs.msg import Image


class HeadDisplay(object):
    """
    Interface class for head display

    Displays a given image file or multiple files on the robot's face.

    Pass the relative or absolute file path to an image file on your
    computer, and the example will read and convert the image using
    cv_bridge, sending it to the screen as a standard ROS Image Message.

    Notes:
    Max screen resolution is 1024x600.
    Images are always aligned to the top-left corner.
    Image formats are those supported by OpenCv - LoadImage().
    """

    def __init__(self):
        """
        Constructor
        """
        self._image_pub = rospy.Publisher('/robot/head_display', Image, latch=True, queue_size=10)

    def _setup_image(self, image_path):
        """
        Load the image located at the specified path

        @type image_path: str
        @param image_path: the relative or absolute file path to the image file

        @type image_delay: float
        @param image_delay: time in seconds to wait before publishing image
        """
        if not os.access(image_path, os.R_OK):
            rospy.logerr("Cannot read file at '{0}'".format(image_path))
            return None

        img = cv2.imread(image_path)
        # Return msg
        return cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")

    def display_image(self, image_path, display_in_loop=False, display_rate=1.0):
        """
        Displays image(s) to robot's head

        @type image_path: list
        @param path: the relative or absolute file path to the image file(s)

        @type display_in_loop: bool
        @param display_in_loop: loop to go through all image file or not

        @type display_rate: float
        @param display_rate: the rate to publish image into head
        """
        rospy.logdebug("Display images in loop:'{0}', frequency: '{1}'".format(display_in_loop, display_rate))

        image_msg = []
        image_list = image_path if isinstance(image_path, list) else [image_path]
        for one_path in image_list:
            cv_img = self._setup_image(one_path)
            if cv_img:
                image_msg.append(cv_img)

        if not image_msg:
            rospy.logerr("Image message list is empty")
        else:
            r = rospy.Rate(display_rate)
            while not rospy.is_shutdown():
                for one_msg in image_msg:
                    self._image_pub.publish(one_msg)
                    r.sleep()
                if not display_in_loop:
                    break