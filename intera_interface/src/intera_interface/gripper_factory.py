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
from intera_core_msgs.msg import (
    IONodeConfiguration,
    IONodeStatus,
    IOComponentConfiguration
)
import intera_dataflow
from intera_interface import (
    Gripper,
    SimpleClickSmartGripper
)


def get_current_gripper_interface():
    """
    Instantiate and return an interface to control the gripper
    currently attached to the robot.

    Looks for the type of the first detected gripper attached to the robot
    and returns an instance of a Gripper or SimpleClickSmartGripper class
    configured to control the EE.

    @rtype: Gripper|SimpleClickSmartGripper
    @return: instance of an interface to control attached gripper
    """
    gf = GripperFactory()
    return gf.get_current_gripper_interface()


class GripperFactory(object):
    """
    Helper class for creating a Gripper interface for the End Effector
    currently connected to the robot. Use the get_current_gripper_interface() fn.

    Listens to the end_effector IO Node for a list of End Effectors currently
    attached. The fn get_current_gripper_interface() will determine the type of the
    first currently connected gripper and return an instance of a corresponding
    SDK Gripper class to control the gripper.
    """
    def __init__(self):
        self.states = []
        self.configs = []
        self._node_state = None
        self._node_config = None
        self._node_state_sub = rospy.Subscriber('/io/end_effector/state', IONodeStatus, self._node_state_cb)
        self._node_config_sub = rospy.Subscriber('/io/end_effector/config', IONodeConfiguration, self._node_config_cb)

        intera_dataflow.wait_for(
            lambda: self._node_state is not None,
            timeout=5.0,
            timeout_msg=("Failed to connect to end_effector IO Node.")
        )

    def _node_state_cb(self, msg):
        self._node_state = msg
        if msg.devices:
            self.states = msg.devices

    def _node_config_cb(self, msg):
        self._node_config = msg
        if msg.devices:
            self.configs = msg.devices

    def _lookup_gripper_class(self, ee_type):
        EE_CLASS_MAP = dict({
            "SmartToolPlate": SimpleClickSmartGripper,
            "ElectricParallelGripper": Gripper,
            "default": Gripper
        })

        return EE_CLASS_MAP.get(ee_type, None) or EE_CLASS_MAP["default"]

    def _parse_config(self, config):
        ee_config = config
        if type(ee_config) == IOComponentConfiguration:
            ee_config = ee_config.config
        if type(ee_config) == str:
            ee_config = json.loads(ee_config)
        if type(ee_config) == dict:
            return ee_config

    def get_current_gripper_interface(self):
        """
        Instantiate and return an interface to control the gripper
        currently attached to the robot.

        Looks for the type of the first detected gripper attached to the robot
        and returns an instance of a Gripper or SimpleClickSmartGripper class
        configured to control the EE.

        @rtype: Gripper|SimpleClickSmartGripper
        @return: instance of an interface to control attached gripper
        """
        gripper = None

        if len(self.states) <= 0 or len(self.configs) <= 0:
            # wait a moment for any attached grippers to populate
            intera_dataflow.wait_for(
                lambda: (len(self.states) > 0 and len(self.configs) > 0),
                timeout=5.0,
                timeout_msg=("Failed to get gripper. No gripper attached on the robot.")
            )

        ee_state = self.states[0]
        ee_id = ee_state.name
        ee_config = None
        for device in self.configs:
            if device.name == ee_id:
                ee_config = self._parse_config(device.config)
                break

        gripper_class = self._lookup_gripper_class(ee_config['props']['type'])
        needs_init = (ee_state.status.tag == 'down' or ee_state.status.tag == 'unready')
        try:
            gripper = gripper_class(ee_id, needs_init)
        except:
            gripper = gripper_class(ee_id, False)

        return gripper
