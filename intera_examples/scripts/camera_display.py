#!/usr/bin/env python

# Copyright (c) 2013-2017, Rethink Robotics Inc.
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

import argparse
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

import rospy
import intera_interface

def show_image_callback(img_data, (edge_detection, window_name)):
    """The callback function to show image by using CvBridge and cv
    """
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8")
    except CvBridgeError, err:
        rospy.logerr(err)
        return
    if edge_detection == True:
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)
        # customize the second and the third argument, minVal and maxVal
        # in function cv2.Canny if needed
        get_edge = cv2.Canny(blurred, 10, 100)
        cv_image = np.hstack([get_edge])
    edge_str = "(Edge Detection)" if edge_detection else ''
    cv_win_name = ' '.join([window_name, edge_str])
    cv2.namedWindow(cv_win_name, 0)
    # refresh the image on the screen
    cv2.imshow(cv_win_name, cv_image)
    cv2.waitKey(3)

def main():
    """Camera Display Example
    """
    rp = intera_interface.RobotParams()
    valid_cameras = rp.get_camera_names()
    if not valid_cameras:
        rp.log_message(("Cannot detect any camera_config"
            " parameters on this robot. Exiting."), "ERROR")
        return
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        '-c', '--camera', type=str, default="head_camera",
        choices=valid_cameras, help='Setup Camera Name for Camera Display')
    parser.add_argument(
        '-r', '--raw', action='store_true', 
        help='Specify use of the raw image (unrectified) topic')
    parser.add_argument(
        '-e', '--edge', action='store_true',
        help='Streaming the Canny edge detection image')
    args = parser.parse_args()

    print("Initializing node... ")
    rospy.init_node('camera_display', anonymous=True)
    camera = intera_interface.Cameras()
    if not camera.verify_camera_exists(args.camera):
        rospy.logerr("Invalid camera name, exiting the example.")
        return
    camera.start_streaming(args.camera)
    rectify_image = not args.raw
    use_canny_edge = args.edge
    camera.set_callback(args.camera, show_image_callback,
        rectify_image=rectify_image, callback_args=(use_canny_edge, args.camera))

    def clean_shutdown():
        print("Shutting down camera_display node.")
        cv2.destroyAllWindows()

    rospy.on_shutdown(clean_shutdown)
    rospy.loginfo("Camera_display node running. Ctrl-c to quit")
    rospy.spin()


if __name__ == '__main__':
    main()
