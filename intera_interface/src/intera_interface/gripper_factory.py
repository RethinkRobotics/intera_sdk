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
    IONodeConfiguration,
    IONodeStatus,
    IOComponentConfiguration
)
import intera_dataflow

# Gripper
# SimpleIOGripper

class GripperFactory(object):

    def __init__(self, live=True):
        self.states = []
        self.configs = []
        # node interface
        self._node_state_sub = rospy.Subscriber('/io/end_effector/state', IONodeStatus, self._node_state_cb)
        self._node_config_sub = rospy.Subscriber('/io/end_effector/config', IONodeConfiguration, self._node_config_cb)

        if live:
            intera_dataflow.wait_for(
                lambda: not self.states is None,
                timeout=5.0, raise_on_error=False,
                timeout_msg=("Failed to get gripper. No gripper attached on the robot.")
            )

    def _node_state_cb(self, msg):
        self.node_state = msg
        if msg.devices:
            self.states = msg.devices

    def _node_config_cb(self, msg):
        self.node_config = msg
        if msg.devices:
            self.configs = msg.devices

    def lookupGripperClass(self, ee_type):
        EE_CLASS_MAP = dict({
            "SmartToolPlate": SimpleIOGripper,
            "ElectricParallelGripper": Gripper,
            "default": Gripper
        })

        return EE_CLASS_MAP.get(ee_type, None) or EE_CLASS_MAP["default"]

    def parseConfig(self, config):
        ee_config = config
        if type(ee_config) == IOComponentConfiguration:
            ee_config = ee_config.config
        if type(ee_config) == str:
            ee_config = json.loads(ee_config)
        if type(ee_config) == dict:
            return ee_config

    def getCurrentGripperInterface(self):

        if len(self.states) <= 0:
            # wait a second
            intera_dataflow.wait_for(
                lambda: (len(self.states) > 0 and len(self.configs) > 0),
                timeout=5.0, raise_on_error=False,
                timeout_msg=("Failed to get gripper. No gripper attached on the robot.")
            )

        if len(self.states) and len(self.configs):
            ee_state = self.states[0]
            ee_id = ee_state.name
            ee_config = None
            for conf in self.configs:
                if conf.name == ee_id:
                    ee_config = self.parseConfig(conf.config)
                    break

            needs_init = True if (ee_state.status.tag == 'down' or ee_state.status.tag == 'unready') else False

            gripper_class = self.lookupGripperClass(ee_config['props']['type'])

            gripper = None
            try:
                gripper = gripper_class(ee_id, needs_init)
            except:
                gripper = gripper_class(ee_id, False)
            return gripper
        else:
            return None
