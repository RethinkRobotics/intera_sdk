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

import json
import rospy
from intera_core_msgs.msg import (
    IONodeStatus,
    IOComponentCommand
)
import intera_dataflow
from intera_io import IODeviceInterface
from intera_io.io_command import IOCommand


class SimpleClickSmartGripper(object):
    """
    Bare bones Interface class for a gripper with a chip on the Intera Research Robot.
    """
    def __init__(self, ee_device_id, initialize=True):
        self.name = ee_device_id
        self.endpoint_map = None
        self.gripper_io = None
        self._node_state = None
        self._node_device_status = None

        self._node_command_pub = rospy.Publisher('io/end_effector/command', IOComponentCommand, queue_size=10)
        self._node_state_sub = rospy.Subscriber('io/end_effector/state', IONodeStatus, self._node_state_cb)

        # Wait for the gripper device status to be true
        intera_dataflow.wait_for(
            lambda: self._node_device_status is not None,
            timeout=5.0, raise_on_error=initialize,
            timeout_msg=("Failed to get gripper. No gripper attached on the robot.")
        )

        self.gripper_io = IODeviceInterface("end_effector", self.name)
        self.gripper_io.config_changed.connect(self._load_endpoint_info)
        if self.gripper_io.is_config_valid():
            self._load_endpoint_info()

        if initialize and self.needs_init():
            self.initialize()

    def _node_state_cb(self, msg):
        # if we don't have a message yet, or if the timestamp is different: search for our device
        if not self._node_state or IODeviceInterface.time_changed(self._node_state.time, msg.time):
            self._node_state = msg
            if len(msg.devices) and msg.devices[0].name == self.name:
                self._node_device_status = msg.devices[0].status
            else:
                # device removed
                self._node_device_status = None

    def _load_endpoint_info(self):
        self._device_config = json.loads(self.config.device.config)
        self.endpoint_map = self._device_config['params']['endpoints']

    def initialize(self, timeout=5.0):
        # activate tool
        cmd = IOCommand('activate', {"devices": [self.name]})
        msg = cmd.as_msg()
        self._node_command_pub.publish(msg)
        if timeout:
            intera_dataflow.wait_for(
                lambda: self.is_ready(), timeout=timeout,
                timeout_msg=("Failed to initialize gripper.")
            )

    def is_ready(self):
        return (self._node_device_status and self._node_device_status.tag == 'ready'
            and self.gripper_io.is_valid())

    def needs_init(self):
        return (self._node_device_status and (self._node_device_status.tag == 'down'
            or self._node_device_status.tag == 'unready'))

    def get_ee_signal_value(self, ee_signal_type, endpoint_id=None):
        (ept_id, endpoint_info) = self.get_endpoint_info(endpoint_id)
        if ee_signal_type in endpoint_info:
            return self.get_signal_value(endpoint_info[ee_signal_type])

    def set_ee_signal_value(self, ee_signal_type, value, endpoint_id=None, timeout=5.0):
        (ept_id, endpoint_info) = self.get_endpoint_info(endpoint_id)
        if ee_signal_type in endpoint_info:
            return self.set_signal_value(endpoint_info[ee_signal_type], value)

    def get_ee_signals(self, endpoint_id=None):
        (ept_id, endpoint_info) = self.get_endpoint_info(endpoint_id)
        exclude = ['endpoint_id', 'label', 'type', 'actuationTimeS']
        return {k: v for k,v in endpoint_info.iteritems() if k not in exclude}

    def get_all_ee_signals(self):
        info = dict()
        for ept in self.list_endpoint_names():
            info[ept] = self.get_ee_signals(ept)
        return info

    def get_all_signals(self):
        return self.gripper_io.signals

    def list_endpoint_names(self):
        if self.endpoint_map:
            return self.endpoint_map.keys()
        else:
            return []

    def get_endpoint_info(self, endpoint_id=None):
        if self.endpoint_map is None or len(self.endpoint_map.keys()) <= 0:
            rospy.logerr('Cannot use endpoint signals without any endpoints!')
            return
        endpoint_id = self.endpoint_map.keys()[0] if endpoint_id is None else endpoint_id
        return (endpoint_id, self.endpoint_map[endpoint_id])

    # proxy-pass through
    def __getattr__(self, name):
        return getattr(self.gripper_io, name)
