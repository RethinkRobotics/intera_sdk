# Copyright (c) 2023, Rethink Robotics GmbH.
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

from intera_interface import (
    gripper_factory
)

from motor_control_msgs.msg import StringArray


class TipSelector(object):
    """
    Define which tips are available via Limb.tip_state(tip_name).

    The default tip is called `right_hand` but other tips like the `right_hand_camera` or
    the STP tip are also available. With the help of this class you can control which tip states
    are available.
    """

    def __init__(self, limb="right", default_tip="hand", camera_tip="hand_camera"):
        """
        Constructor.

        @type limb: str
        @param limb: limb to interface

        @type default_tip: str
        @param default_tip: default tip name

        @type camera_tip: str
        @param camera_tip: default camera tip name
        """
        self._limb = limb
        self._hand_tip = self._limb + "_" + default_tip
        self._camera_tip = self._limb + "_" + camera_tip
        self._available_tips = [self._hand_tip, self._camera_tip]
        self._selected_tips = [self._hand_tip]

        self._pub_endpoints = rospy.Publisher(
            '/intera/endpoint_ids',
            StringArray,
            latch=True,
            queue_size=10)

    def add_hand_tip(self):
        if self._hand_tip not in self._selected_tips:
            self._selected_tips.append(self._hand_tip)
        self._publish()

    def add_camera_tip(self):
        if self._camera_tip not in self._selected_tips:
            self._selected_tips.append(self._camera_tip)
        self._publish()

    def scan_stp(self):
        """Scans for available Smart Tool Plates (STP) and returns a list of all available tips.

        @rtype: list(str)
        @return: The names of all available tips
        """
        gripper = gripper_factory.get_current_gripper_interface()
        for tips in gripper.endpoint_map:
            self._available_tips.append(gripper.endpoint_map[tips]['endpoint_id'])
        return self._available_tips

    def add_tip(self, tip_name):
        if tip_name not in self._available_tips:
            rospy.logerr(f"Tip with name {tip_name} is not available!")
            return False
        if tip_name not in self._selected_tips:
            self._selected_tips.append(self._camera_tip)
        self._publish()
        return True

    def remove_tip(self, tip_name):
        if tip_name not in self._selected_tips:
            rospy.logerr(f"Tip with name {tip_name} is not available!")
            return False
        self._selected_tips.remove(tip_name)
        self._publish()
        return True

    def _publish(self):
        msg = StringArray()
        msg.data = [tip for tip in self._selected_tips]
        self._pub_endpoints.publish(msg)
