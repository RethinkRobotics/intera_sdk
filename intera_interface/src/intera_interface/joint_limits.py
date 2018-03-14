# Copyright (c) 2018, Rethink Robotics Inc.
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

from copy import deepcopy
import rospy
from intera_core_msgs.msg import JointLimits as JointLimitsMsg
import intera_dataflow

class JointLimits(object):
    """
    Interface class for joint limits on Intera robots
    """

    def __init__(self):
        """
        Constructor
        """
        self._joint_position_lower = dict()
        self._joint_position_upper = dict()
        self._joint_velocity_limit = dict()
        self._joint_accel_limit = dict()
        self._joint_effort_limit = dict()
        self._joint_names = list()

        joint_limit_topic = '/robot/joint_limits'
        _joint_limit_sub = rospy.Subscriber(
            joint_limit_topic,
            JointLimitsMsg,
            self._on_joint_limits,
            queue_size=1)

        err_msg = ("%init failed to get current joint_limits "
                   "from %s", joint_limit_topic)
        intera_dataflow.wait_for(lambda: len(self._joint_names) > 0,
                                 timeout_msg=err_msg, timeout=5.0)

    def _on_joint_limits(self, msg):
        self._joint_names = msg.joint_names
        self._joint_position_lower.clear()
        self._joint_position_upper.clear()
        self._joint_velocity_limit.clear()
        self._joint_accel_limit.clear()
        self._joint_effort_limit.clear()
        for idx, name in enumerate(msg.joint_names):
            self._joint_position_lower[name] = msg.position_lower[idx]
            self._joint_position_upper[name] = msg.position_upper[idx]
            self._joint_velocity_limit[name] = msg.velocity[idx]
            self._joint_accel_limit[name] = msg.accel[idx]
            self._joint_effort_limit[name] = msg.effort[idx]

    def joint_position_lower_limits(self):
        """
        Return the joint lower position limits  (radians)

        @rtype: dict({str:float})
        @return: unordered dict of joint lower position limits
        """
        return deepcopy(self._joint_position_lower)

    def joint_position_upper_limits(self):
        """
        Return the joint upper position limits  (radians)

        @rtype: dict({str:float})
        @return: unordered dict of joint upper position limits
        """
        return deepcopy(self._joint_position_upper)

    def joint_velocity_limits(self):
        """
        Return the joint velocity limits  (rad/s)

        @rtype: dict({str:float})
        @return: unordered dict of joint velcoity limits
        """
        return deepcopy(self._joint_velocity_limit)

    def joint_acceleration_limits(self):
        """
        Return the joint acceleration limits  (rad/s/s)

        @rtype: dict({str:float})
        @return: unordered dict of joint accleration limits
        """
        return deepcopy(self._joint_accel_limit)

    def joint_effort_limits(self):
        """
        Return the joint effort limits  (Nm)

        @rtype: dict({str:float})
        @return: unordered dict of joint effort limits
        """
        return deepcopy(self._joint_effort_limit)

    def joint_lower_limit(self, joint):
        """
        Return the requested joint lower position limit

        @type joint: str
        @param joint: name of a joint
        @rtype: float
        @return: joint lower position limit  (radian)
        """
        return self._joint_position_lower[joint]

    def joint_upper_limit(self, joint):
        """
        Return the requested joint upper position limit

        @type joint: str
        @param joint: name of a joint
        @rtype: float
        @return: joint upper position limit  (radian)
        """
        return self._joint_position_upper[joint]

    def joint_velocity_limit(self, joint):
        """
        Return the requested joint velocity limit

        @type joint: str
        @param joint: name of a joint
        @rtype: float
        @return: joint velocity limit  (rad/s)
        """
        return self._joint_velocity_limit[joint]

    def joint_acceleration_limit(self, joint):
        """
        Return the requested joint accleration limit

        @type joint: str
        @param joint: name of a joint
        @rtype: float
        @return: joint acceleration limit  (rad/s/s)
        """
        return self._joint_accel_limit[joint]

    def joint_effort_limit(self, joint):
        """
        Return the requested joint effort limit

        @type joint: str
        @param joint: name of a joint
        @rtype: float
        @return: joint effort limit  (Nm)
        """
        return self._joint_effort_limit[joint]

    def get_joint_lower_limits(self, joint_names):
        """
        Return the joint lower position limits  (radians)

        @type joint_names: [str]
        @param joint_names: list of joint names
        @rtype: [float]
        @return: ordered list of joint lower position limits
        """
        return [self._joint_position_lower[name] for name in joint_names]

    def get_joint_upper_limits(self, joint_names):
        """
        Return the joint upper position limits  (radians)

        @type joint_names: [str]
        @param joint_names: list of joint names
        @rtype: [float]
        @return: ordered list of joint upper position limits
        """
        return [self._joint_position_upper[name] for name in joint_names]

    def get_joint_velocity_limits(self, joint_names):
        """
        Return the joint velocity limits  (rad/s)

        @type joint_names: [str]
        @param joint_names: list of joint names
        @rtype: [float]
        @return: ordered list of joint velocity limits
        """
        return [self._joint_velocity_limit[name] for name in joint_names]

    def get_joint_acceleration_limits(self, joint_names):
        """
        Return the joint acceleration limits  (rad/s/s)

        @type joint_names: [str]
        @param joint_names: list of joint names
        @rtype: [float]
        @return: ordered list of joint acceleration limits
        """
        return [self._joint_accel_limit[name] for name in joint_names]

    def get_joint_effort_limits(self, joint_names):
        """
        Return the joint effort limits  (Nm)

        @type joint_names: [str]
        @param joint_names: list of joint names
        @rtype: [float]
        @return: ordered list of joint effort limits
        """
        return [self._joint_effort_limit[name] for name in joint_names]

