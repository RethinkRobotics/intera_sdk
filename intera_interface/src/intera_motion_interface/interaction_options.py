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
from intera_core_msgs.msg import InteractionControlCommand
from geometry_msgs.msg import Pose, Point, Quaternion
from copy import deepcopy
from utility_functions import (
    ensure_path_to_file_exists,
    clamp_float_warn
)
from rospy_message_converter import message_converter
import yaml

class InteractionOptions(object):
    """
    This class is a wrapper for the intera message:  InteractionControlCommand.msg
    It's primary purpose is to facilitate message creation and input checking.
    """

    n_dim_cart = 6 # number of dimensions in Cartesian space

    # constants for interaction control modes
    force_mode = InteractionControlCommand.FORCE_MODE
    impedance_mode = InteractionControlCommand.IMPEDANCE_MODE
    impedance_limit_mode = InteractionControlCommand.IMPEDANCE_WITH_FORCE_LIMIT_MODE
    force_limit_mode = InteractionControlCommand.FORCE_WITH_MOTION_LIMIT_MODE

    # default parameters
    default_n_dim_joint = 7
    default_K_impedance_lateral = 1300  # N/m
    default_K_impedance_rotation = 30   # Nm/rad
    default_K_nullspace = 4.0  # Nm/rad
    default_endpoint_name = 'right_hand'
    default_max_impedance = True
    default_in_endpoint_frame = False
    default_force_command = 0.0
    default_interaction_mode = impedance_mode
    default_disable_damping_in_force_control = False
    default_disable_reference_resetting = False

    def __init__(self, header = None,
                 interaction_control_active = None,
                 K_impedance = None,
                 max_impedance = None,
                 n_joints = None,
                 K_nullspace = None,
                 force_command= None,
                 interaction_frame = None,
                 endpoint_name = None,
                 in_endpoint_frame = None,
                 interaction_control_mode = None,
                 disable_damping_in_force_control = None,
                 disable_reference_resetting = None):
        """
        Create a interaction options object. All parameters are
        optional. If ommitted or set to None, then use default value.

        See setter functions for more specific details.
        """
        self._data = InteractionControlCommand()
        self.set_header(header)
        self.set_interaction_control_active(interaction_control_active)
        self.set_number_joints(n_joints),
        self.set_K_impedance(K_impedance)
        self.set_max_impedance(max_impedance)
        self.set_K_nullspace(K_nullspace)
        self.set_force_command(force_command)
        self.set_interaction_frame(interaction_frame)
        self.set_endpoint_name(endpoint_name)
        self.set_in_endpoint_frame(in_endpoint_frame)
        self.set_interaction_control_mode(interaction_control_mode)
        self.set_disable_damping_in_force_control(disable_damping_in_force_control)
        self.set_disable_reference_resetting(disable_reference_resetting)

    def set_header(self, header = None):
        """
        @param header:
          - None: set default header with current timestamp
          - [header]: set message header
        """
        if header is None:
            self._data.header.stamp = rospy.get_rostime()
            self._data.header.seq = 1
            self._data.header.frame_id = "base"
        else:
            self._data.header = header

    def set_interaction_control_active(self, interaction_active = None):
        """
        @param interaction_control_active:
          - None:  set it to True by default
          - [bool]: copy the element.
        """
        if interaction_active is None:
            self.set_interaction_control_active(True) # default value
        else:
            self._data.interaction_control_active = interaction_active

    def set_K_impedance(self, K_impedance = None):
        """
        @param K_impedance (Cartesian stiffness):
          - None:  populate with vector of default values
          - [float]:  copy all elements. Checks length.
        """
        if K_impedance is None:
            self._data.K_impedance = [self.default_K_impedance_lateral]*3 \
                                     + [self.default_K_impedance_rotation]*3
        elif len(K_impedance) == self.n_dim_cart:
            self._data.K_impedance = deepcopy(K_impedance)
        else:
            rospy.logerr('The number of K Impedance elements must be %d',
                         self.n_dim_cart)

    def set_max_impedance(self, max_impedance = None):
        """
        @param max_impedance (impedance modulation state):
          - None:  populate with vector of default values (True)
          - bool:  set each dimension by the input boolean
          - [bool]:  copy all elements. Checks length.
        """
        if max_impedance is None:
            self._data.max_impedance =  [self.default_max_impedance]*self.n_dim_cart
        elif len(max_impedance) == 1:
            self._data.max_impedance = [max_impedance[0]]*self.n_dim_cart
        elif len(max_impedance) == self.n_dim_cart:
            self._data.max_impedance = deepcopy(max_impedance)
        else:
            rospy.logerr('The number of max_impedance must be 1 or %d',
                         self.n_dim_cart)

    def set_number_joints(self, n_joints = None):
        """
        @param n_joints (number of arm joints):
          - None:  use default number
          - [int]: use provided number
        """
        if n_joints is None:
            self._n_dim_joint = self.default_n_dim_joint
        else:
            self._n_dim_joint = n_joints

    def set_K_nullspace(self, K_nullspace = None):
        """
        @param K_nullspace (nullspace stiffness gains):
          - None:  populate with vector of default values
          - float:  set each dimension by the input float number
          - [float]:  copy all elements. Checks length.
        """
        if K_nullspace is None:
            self._data.K_nullspace = [self.default_K_nullspace]*self._n_dim_joint
        elif len(K_nullspace) == 1:
            self._data.K_nullspace = [K_nullspace[0]]*self._n_dim_joint
        elif len(K_nullspace) == self._n_dim_joint:
            self._data.K_nullspace = deepcopy(K_nullspace)
        else:
            rospy.logerr('The number of K_nullspace must be 1 or %d',
                         self._n_dim_joint)

    def set_force_command(self, force_cmd = None):
        """
        @param force_command:
          - None:  populate with vector of default values
          - [float]:  copy all elements. Checks length.
        """
        if force_cmd is None:
            self._data.force_command =  [self.default_force_command]*self.n_dim_cart
        elif len(force_cmd) == self.n_dim_cart:
            self._data.force_command = deepcopy(force_cmd)
        else:
            rospy.logerr('The number of force_command elements must be %d',
                         self.n_dim_cart)

    def set_interaction_frame(self, interaction_frame = None):
        """
        @param interaction_frame:
          - None:  populate with vector of default values
          - Pose: copy all the elements
        """
        if interaction_frame is None:
            self._data.interaction_frame = Pose(
                position=Point(x=0.0,y=0.0,z=0.0),
                orientation=Quaternion(x=0.0,y=0.0,z=0.0,w=1.0))
        elif isinstance(interaction_frame, Pose):
            self._data.interaction_frame = interaction_frame
        else:
            rospy.logerr('interaction_frame must be of type geometry_msgs.Pose')

    def set_endpoint_name(self, endpoint_name = None):
        """
        @param endpoint_name:
          - None:  set it to a default name (right hand)
          - string: copy element.
        """
        if endpoint_name is None:
            self._data.endpoint_name = self.default_endpoint_name
        else:
            self._data.endpoint_name = endpoint_name

    def set_in_endpoint_frame(self, in_endpoint_frame = None):
        """
        @param in_endpoint_frame:
          - None:  set it to a default frame (False, i.e., base frame)
          - bool:  copy element
        """
        if in_endpoint_frame is None:
            self._data.in_endpoint_frame = self.default_in_endpoint_frame
        else:
            self._data.in_endpoint_frame = in_endpoint_frame

    def set_interaction_control_mode(self, interaction_mode = None):
        """
        @param interaction_mode:
          - None:  set impedance mode by default
          - mode:  set each direction by the input mode
            (1: impedance, 2: force, 3: impedance with force limit, 4: force with motion limit)
          - [mode]:  copy all elements. Checks length.
        """
        if interaction_mode is None:
            self._data.interaction_control_mode = [self.default_interaction_mode]*self.n_dim_cart
        else:
            # first check for valid modes
            for i in range(len(interaction_mode)):
                if (interaction_mode[i] < self.impedance_mode
                    or interaction_mode[i] > self.force_limit_mode):
                    rospy.logerr('Interaction mode option %d is invalid', interaction_mode[i])
                    return

            if len(interaction_mode) == 1:
                self._data.interaction_control_mode = [interaction_mode[0]]*self.n_dim_cart
            elif len(interaction_mode) == self.n_dim_cart:
                self._data.interaction_control_mode = deepcopy(interaction_mode)
            else:
                rospy.logerr('The number of interaction_control_mode elements must be 1 or %d',
                             self.n_dim_cart)

    def set_disable_damping_in_force_control(self, disable_damping_in_force_control = None):
        """
        @param disable_damping_in_force_control:
          - None:  set it to False by default
          - [bool]: copy the element.
        """
        if disable_damping_in_force_control is None:
            self.set_disable_damping_in_force_control(self.default_disable_damping_in_force_control) # default value
        else:
            self._data.disable_damping_in_force_control = disable_damping_in_force_control

    def set_disable_reference_resetting(self, disable_reference_resetting = None):
        """
        @param disable_reference_resetting:
          - None:  set it to False by default
          - [bool]: copy the element.
        """
        if disable_reference_resetting is None:
            self.set_disable_reference_resetting(self.disable_reference_resetting) # default value
        else:
            self._data.disable_reference_resetting = disable_reference_resetting


    def to_msg(self):
        """
        @return: InteractionControlCommand.msg
        """
        return deepcopy(self._data)

    def to_dict(self):
        """
        @return: the interaction control options as a dictionary object
        """
        return message_converter.convert_ros_message_to_dictionary(self._data)

    def to_string(self):
        """
        @return: a yaml-formatted string with the interaction control options
        """
        return yaml.dump(self.to_dict(), default_flow_style=False)

    def to_yaml_file(self, file_name):
        """
        Write the contents of the interaction control options to a yaml file
        """
        file_name = ensure_path_to_file_exists(file_name)
        with open(file_name, "w") as outfile:
            yaml.dump(self.to_dict(), outfile, default_flow_style=False)
