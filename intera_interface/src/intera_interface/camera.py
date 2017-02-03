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

import rospy
from sensor_msgs.msg import Image
from intera_io import IODeviceInterface
from robot_params import RobotParams

class Cameras(object):
    """
    Base class for the interface to the robot's Cameras.
    """

    def __init__(self):
        """
        Constructor.
        """
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
