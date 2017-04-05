#! /usr/bin/env python

# Copyright (c) 2016-2017, Rethink Robotics Inc.
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
import actionlib
import motion_msgs.msg
from intera_core_msgs.msg import (
    JointLimits,
    DigitalIOState,
    EndpointState,
    EndpointStates
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from copy import deepcopy
from motion_interface.utility_functions import wait_for

class RobotInterface(object):

    """
    NOTE: This class should eventually be removed and replaced with the
    limb.py interface from the SDK
    """

    def __init__(self):
        """
        Constructor - creates a new robot interface
        """
        timeout = 5.0

        self._skip_trajectory_pub = rospy.Publisher(
            '/motion/skip_trajectory', Bool, queue_size=5)

        self._joint_limits = None
        self._joint_limits_sub = rospy.Subscriber(
            '/robot/joint_limits',
            JointLimits,
            self._on_joint_limits)
        wait_for(lambda: self._joint_limits is not None, timeout=timeout)

        self._joint_states = None
        self._joint_states_sub = rospy.Subscriber(
            '/robot/joint_states',
            JointState,
            self._on_joint_state)
        wait_for(lambda: self._joint_states is not None, timeout=timeout)

        self._in_sim_mode = None
        self._in_sim_mode_sub = rospy.Subscriber(
            '/robot/in_sim_mode',
            Bool,
            self._on_in_sim_mode)
        wait_for(lambda: self._in_sim_mode is not None, timeout=timeout)

        # default endpoint state
        self._endpoint_state = None
        self._endpoint_state_sub = rospy.Subscriber(
            '/robot/limb/right/endpoint_state',
            EndpointState,
            self._on_endpoint_state)
        wait_for(lambda: self._endpoint_state is not None, timeout=timeout)

        # all endpoint states
        self._tip_states = None
        self._tip_states_sub = rospy.Subscriber(
            '/robot/limb/right/tip_states',
            EndpointStates,
            self._on_tip_states)
        wait_for(lambda: self._tip_states is not None, timeout=timeout)


    def _on_joint_limits(self, msg):
        self._joint_limits = deepcopy(msg)

    def get_joint_limits(self, exclude_joints = ['head_pan']):
        if exclude_joints:
            limits = JointLimits()
            lim = self._joint_limits
            for i in range(0,len(lim.joint_names)):
                if lim.joint_names[i] not in exclude_joints:
                    limits.joint_names.append(lim.joint_names[i])
                    limits.position_lower.append(lim.position_lower[i])
                    limits.position_upper.append(lim.position_upper[i])
                    limits.velocity.append(lim.velocity[i])
                    limits.accel.append(lim.accel[i])
                    limits.effort.append(lim.effort[i])
        else:
            limits = deepcopy(self._joint_limits)
        return limits

    def _on_joint_state(self, msg):
        self._joint_states = deepcopy(msg)

    def get_joint_state(self, exclude_joints = ['head_pan', 'torso_t0']):
        if exclude_joints:
            states = JointState()
            for (pos, name) in zip(self._joint_states.position,
                                  self._joint_states.name):
                if name not in exclude_joints:
                    states.position.append(pos)
                    states.name.append(name)
        else:
            states = deepcopy(self._joint_states)
        return states

    def _on_in_sim_mode(self, msg):
        self._in_sim_mode = deepcopy(msg.data)

    def get_in_sim_mode(self):
        return deepcopy(self._in_sim_mode)

    def _on_endpoint_state(self, msg):
        self._endpoint_state = deepcopy(msg)

    def get_endpoint_state(self):
        return deepcopy(self._endpoint_state)

    def _on_tip_states(self, msg):
        self._tip_states = deepcopy(msg)

    def get_all_endpoint_states(self):
        return deepcopy(self._tip_states)

    def get_endpoint_state(self, tipName):
        if tipName is None:
           indx = 0
        else:
           try:
              indx = self._tip_states.names.index(tipName)
           except ValueError:
              return None
        return deepcopy(self._tip_states.states[indx])

    def set_skip_trajectory(self, skip_trajectory):
        if not self._in_sim_mode and skip_trajectory:
            rospy.logwarn("Ignoring request: can only skip trajectories in simulation.")
            return False
        self._skip_trajectory_pub.publish(skip_trajectory)
        if skip_trajectory:
            rospy.loginfo("Setting skip_trajectory to True")
        else:
            rospy.loginfo("Setting skip_trajectory to False")
        return True
