# Copyright (c) 2013-2018, Rethink Robotics Inc.
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

        @rtype: sensor_msgs/Image or None
        @param: Returns sensor_msgs/Image if image convertable and None otherwise
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
        @param image_path: the relative or absolute file path to the image file(s)

        @type display_in_loop: bool
        @param display_in_loop: Set True to loop through all image files infinitely

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
