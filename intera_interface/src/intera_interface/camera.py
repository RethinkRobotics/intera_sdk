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


import rospy
from sensor_msgs.msg import Image
from io_interface import IODeviceInterface
from robot_params import RobotParams

class Cameras(object):
    """
    Base class for the interface to the robot's Cameras.
    """

    def __init__(self):
        camera_param_dict = RobotParams().get_camera_details()
        camera_list = camera_param_dict.keys()
        # check to make sure cameras is not an empty list
        if not camera_list:
            rospy.logerr(' '.join(["camera list is empty: ", ' , '.join(camera_list)]))
            return
        camera_color_dict = {"mono":['cognex'], "color":['ienso_ethernet']}
        self.cameras_io = dict()
        for camera in camera_list:
            if camera_param_dict[camera]['cameraType'] in camera_color_dict[''
            'color']:
                is_color = True
            else:
                is_color = False
            self.cameras_io[camera] = {'interface': IODeviceInterface("internal"
                "_camera", camera), 'is_color': is_color}

    def _camera_streaming_status(self, camera_name):
        """
        Private function to check if the camera is currently in streaming mode.

        @type camera_name: str
        @param camera_name: camera name

        @rtype: bool
        @return: True if the camera is streaming, False otherwise
        """
        return self.cameras_io[camera_name]['interface'].get_signal_value(
            "camera_streaming")

    def list_cameras(self):
        """
        Return the list of all camera names on current robot.

        @rtype: [str]
        @return: ordered list of camera names
        """
        return self.cameras_io.keys()

    def verify_camera_exists(self, camera_name):
        """
        Verify if the given camera name is in the list of camera names or not.

        @type camera_name: str
        @param camera_name: camera name

        @rtype: bool
        @return: True if the name exists in camera name list, False otherwise.
        """
        if camera_name  not in self.list_cameras():
            rospy.logerr(' '.join([camera_name, "not in the list of cameras"
                " detected on this robot:", ' , '.join(self.list_cameras())]))
            return False
        return True

    def is_camera_streaming(self, camera_name):
        """
        Check the given camera name is streaming or not.

        @type camera_name: str
        @param camera_name: camera name

        @rtype: bool
        @return: True if the camera is streaming, False camera is not
                 streaming, False with log error means camera name not exists
                 in camera name list
        """
        if self.verify_camera_exists(camera_name):
            return self._camera_streaming_status(camera_name)
        return False

    def set_callback(self, camera_name, callback, callback_args=None,
        queue_size=10, rectify_image=True):
        """
        Setup the callback function to show image.

        @type camera_name: str
        @param camera_name: camera name
        @type callback: fn(msg, cb_args)
        @param callback: function to call when data is received
        @type callback_args: any
        @param callback_args: additional arguments to pass to the callback
        @type queue_size: int
        @param queue_size: maximum number of messages to receive at a time
        @type rectify_image: bool
        @param rectify_image: specify whether subscribe to the rectified or
                              raw (unrectified) image topic
        """
        if self.verify_camera_exists(camera_name):
            if rectify_image == True:
                if self.cameras_io[camera_name]['is_color']:
                    image_string = "image_rect_color"
                else:
                    image_string = "image_rect"
            else:
                image_string = "image_raw"
            rospy.Subscriber('/'.join(["/io/internal_camera", camera_name,
                image_string]), Image, callback, callback_args=callback_args)

    def start_streaming(self, camera_name):
        """
        Start camera streaming for the given camera name, This only allows
        one camera open at one time and forces closed any other open cameras
        before open the wanted one.

        @type camera_name: str
        @param camera_name: camera name

        @rtype: bool
        @return: False if camera not exists in camera_name_list or the
                 interface is not able to stop streaming other camera.
                 Additionally, returns False if the interface is not able
                 to start streaming the desired camera. Returns True if the
                 camera already streaming or the camera successfully start
                 streaming.
        """
        if not self.verify_camera_exists(camera_name):
            return False
        elif not self._camera_streaming_status(camera_name):
            other_cameras_list = list(set(self.list_cameras())-set([
                camera_name]))
            for other_camera in other_cameras_list:
                if self._camera_streaming_status(other_camera):
                    self.cameras_io[other_camera]['interface'].set_signal_value(
                        "camera_streaming", False)
                    if self._camera_streaming_status(other_camera):
                        rospy.logerr(' '.join(["Failed to stop",
                            other_camera, "from streaming on this robot. "
                            "Unable to start streaming from", camera_name]))
                        return False
            self.cameras_io[camera_name]['interface'].set_signal_value(
                "camera_streaming", True)
            if not self._camera_streaming_status(camera_name):
                rospy.logerr(' '.join(["Failed to start", camera_name, "Unable"
                    " to start streaming from", camera_name]))
                return False
            else:
                return True
        else: # Camera is already streaming
            return True

    def stop_streaming(self, camera_name):
        """
        Stop camera streaming by given the camera name.

        @type camera_name: str
        @param camera_name: camera name

        @rtype: bool
        @return: False if camera not exists in camera name list or not able
                 to stop streaming camera. True if the camera not is streaming
                 mode or the camera successfully stop streaming.
        """
        if not self.verify_camera_exists(camera_name):
            return False
        elif self._camera_streaming_status(camera_name):
            self.cameras_io[camera_name]['interface'].set_signal_value(
                "camera_streaming", False)
            if self._camera_streaming_status(camera_name):
                rospy.logerr(' '.join(["Failed to stop", camera_name,
                " from streaming on this robot."]))
                return False
            else:
                return True
        else: # Camera not in streaming mode
            return True
