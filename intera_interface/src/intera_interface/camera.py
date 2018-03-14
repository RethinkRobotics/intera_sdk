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

import json

import rospy

from sensor_msgs.msg import Image
from intera_core_msgs.msg import IONodeConfiguration

import intera_dataflow
from robot_params import RobotParams
from intera_io import IODeviceInterface


class Cameras(object):
    """
    Base class for the interface to the robot's Cameras.
    """

    def __init__(self):
        """
        Constructor.
        """
        self._node_config = None
        self._node_config_sub = rospy.Subscriber('io/internal_camera/config', IONodeConfiguration, self._node_config_cb)
        self.cameras_io = dict()

        camera_param_dict = RobotParams().get_camera_details()
        camera_list = camera_param_dict.keys()
        # check to make sure cameras is not an empty list
        if not camera_list:
            rospy.logerr(' '.join(["camera list is empty: ", ' , '.join(camera_list)]))
            return

        intera_dataflow.wait_for(
            lambda: self._node_config is not None,
            timeout=5.0,
            timeout_msg=("Failed to connect to Camera Node and retrieve configuration.")
        )
        cameras_to_load = self._get_camera_launch_config().keys()

        camera_capabilities = {
            "mono": ['cognex'],
            "color": ['ienso_ethernet'],
            "auto_exposure": ['ienso_ethernet'],
            "auto_gain": ['ienso_ethernet']
        }
        for camera in camera_list:
            cameraType = camera_param_dict[camera]['cameraType']
            try:
                interface = IODeviceInterface("internal_camera", camera)

                self.cameras_io[camera] = {
                    'interface': interface,
                    'is_color': (cameraType in camera_capabilities['color']),
                    'has_auto_exposure': (cameraType in camera_capabilities['auto_exposure']),
                    'has_auto_gain': (cameraType in camera_capabilities['auto_gain']),
                }
            except OSError as e:
                if camera not in cameras_to_load:
                    rospy.logwarn("Expected camera ({0}) is not configured to launch in this run"
                    " configuration. Make sure you are running in SDK Mode.".format(camera))
                else:
                    rospy.logerr("Could not find expected camera ({0}) for this robot.\n"
                        "Please contact Rethink support: support@rethinkrobotics.com".format(camera))

    def _node_config_cb(self, msg):
        self._node_config = msg

    def _get_camera_launch_config(self):
        """
        Retrieve set of camera params for cameras configured to load in current
        software mode configuration.
        """
        plugins = self._node_config.plugins
        cameras_params = dict()
        for plugin in plugins:
            plugin_config = json.loads(plugin.config)
            if 'params' in plugin_config and 'cameras' in plugin_config['params']:
                for camera in plugin_config['params']['cameras']:
                    cameras_params[camera['name']] = camera
        return cameras_params

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

    def _get_signal_status(self, camera_name, signal_name):
        for signal in self.cameras_io[camera_name]['interface'].state.signals:
            if signal.name == signal_name:
                return signal.status
        return None

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
        if camera_name not in self.list_cameras():
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
                 streaming, False with log error means camera name does not
                 exist in camera name list
        """
        if self.verify_camera_exists(camera_name):
            return self._camera_streaming_status(camera_name)
        return False

    def set_callback(self, camera_name, callback, callback_args=None,
        queue_size=1, buff_size=2**23, rectify_image=True):
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
            Defaults to a queue size of 1 to grab the most recent image
        @type buff_size: int
        @param buff_size: incoming message buffer size in bytes.
            Should be greater than the queue_size times the message size.
            Defaults (8 MB) to be larger than a single head image of 4.1 MB.
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
                image_string]), Image, callback, callback_args=callback_args,
                queue_size=queue_size, buff_size=buff_size)

    def start_streaming(self, camera_name):
        """
        Start camera streaming for the given camera name. This only allows
        one camera open at one time and forces closed any other open cameras
        before open the wanted one.

        @type camera_name: str
        @param camera_name: camera name

        @rtype: bool
        @return: False if camera does not exist in camera_name_list or the
                 interface is not able to stop streaming other camera.
                 Additionally, returns False if the interface is not able
                 to start streaming the desired camera. Returns True if the
                 camera is already streaming or the camera successfully starts
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
        Stop camera streaming for the given camera name.

        @type camera_name: str
        @param camera_name: camera name

        @rtype: bool
        @return: False if camera does not exist in camera name list or not able
                 to stop streaming camera. True if the camera is already not in
                 streaming mode or streaming is successfully stopped.
        """
        if not self.verify_camera_exists(camera_name):
            return False
        elif self._camera_streaming_status(camera_name):
            self.cameras_io[camera_name]['interface'].set_signal_value(
                "camera_streaming", False)
            if self._camera_streaming_status(camera_name):
                rospy.logerr(' '.join(["Failed to stop ", camera_name,
                " from streaming on this robot."]))
                return False
            else:
                return True
        else: # Camera not in streaming mode
            return True

    def get_exposure(self, camera_name):
        """
        Return the current exposure setting for the given camera.

        @type camera_name: str
        @param camera_name: camera name

        @rtype: int|float
        @return: current exposure setting (-1 means auto-exposure was set)
        """
        if self.verify_camera_exists(camera_name):
            return self.cameras_io[camera_name]['interface'].get_signal_value('set_exposure')
        return None

    def get_gain(self, camera_name):
        """
        Return the current gain setting for the given camera.

        @type camera_name: str
        @param camera_name: camera name

        @rtype: int
        @return: current gain setting (-1 means auto-gain was set)
        """
        if self.verify_camera_exists(camera_name):
            return self.cameras_io[camera_name]['interface'].get_signal_value('set_gain')
        return None

    def set_exposure(self, camera_name, exposure):
        """
        Set the exposure on the given camera.

        Cognex hand_camera: Range [0.01-100]
        head_camera: Range [0-100], or -1 for auto-exposure
        Setting auto-exposure will turn off auto-gain (and vice-versa).

        @type camera_name: str
        @param camera_name: camera name
        @type exposure: int|float
        @param exposure: new exposure setting, or -1 for auto-exposure

        @rtype: bool
        @return: False, if camera is not available or the given exposure value
                 is invalid. True, if new exposure setting was sent to camera.
        """
        success = False
        if self.verify_camera_exists(camera_name):
            if exposure == -1 and not self.cameras_io[camera_name]['has_auto_exposure']:
                rospy.logerr("Camera ({0}) does not support auto-exposure".format(camera_name))
            else:
                # send camera signal
                self.cameras_io[camera_name]['interface'].set_signal_value('set_exposure', exposure)

                # check result
                status = self._get_signal_status(camera_name, 'set_exposure')
                success = (status and status.tag == 'ready')
                if not success:
                    rospy.logerr("Problem setting signal: {}".format(status or "Signal Not Found"))

        return success

    def set_gain(self, camera_name, gain):
        """
        Set the gain on the given camera.

        Cognex hand_camera: Range [0-255]
        head_camera: Range [0-79], or -1 for auto-gain
        Note: Setting auto-exposure will turn off auto-gain (and vice-versa).

        @type camera_name: str
        @param camera_name: camera name
        @type gain: int
        @param gain: new gain value, -1 for auto-gain

        @rtype: bool
        @return: False, if camera is not available or the given gain value is
                 invalid. True, if new gain setting was sent to camera.
        """
        success = False
        if self.verify_camera_exists(camera_name):
            if gain == -1 and not self.cameras_io[camera_name]['has_auto_gain']:
                rospy.logerr("Camera ({0}) does not support auto-gain".format(camera_name))
            else:
                self.cameras_io[camera_name]['interface'].set_signal_value('set_gain', gain)

                status = self._get_signal_status(camera_name, 'set_gain')
                success = (status and status.tag == 'ready')
                if not success:
                    rospy.logerr("Problem setting signal: {}".format(status or "Signal Not Found"))

        return success

    def set_cognex_strobe(self, value):
        """
        Set the strobe on the Cognex right_hand_camera only.

        @type value: bool
        @param value: True for strobe on, False for strobe off

        @rtype: bool
        @return: False, if Cognex camera is not available and
                 True, otherwise.
        """
        success = True
        try:
            self.cameras_io['right_hand_camera']['interface'].set_signal_value('set_strobe', bool(value))
        except KeyError as e:
            success = False
            rospy.logerr("Cannot find Cognex camera with the name {}".format(e))
        return success
