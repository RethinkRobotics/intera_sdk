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


import os
import math
import errno
from intera_core_msgs.srv import (
    SolvePositionFK,
    SolvePositionFKRequest,
    SolvePositionIK,
    SolvePositionIKRequest,
)
import rospy
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
import csv
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from copy import deepcopy

def get_formatted_decimal_string(i, n):
    """
    @param i: index
    @param n: total number of indicies
    @return: str(i) with correct number of padding zeros
    """
    n_digits = len(str(n))
    format_string = '{:0>' + str(n_digits) + 'd}'
    return format_string.format(i)


def ensure_path_to_file_exists(raw_file_path):
    """
    This function does two checks:
    1) Expands the ~ in the file path string
    2) Creates any intermediate directories that do not exist
    @param raw_file_path
    """
    file_name = os.path.expanduser(raw_file_path)
    file_dir = os.path.dirname(file_name)
    if not os.path.exists(file_dir):
        os.makedirs(file_dir)
    return file_name

def is_valid_check_list_for_none(list):
    for data in list:
        if not data:
            rospy.logwarn("This list contains at least one None value!")
            return False
    return True

def clamp_float_warn(low, val, upp, name):
    """
    Clamps: low <= val <= upp
    Prints: warning if clamped, error if input is not a float
    @param low: lower bound for the input  (float)
    @param val: input (float ?)
    @param upp: upper bound for the input  (float)
    @param name: input name (str)
    @return: clamp(low,val,upp) if input is a float, None otherwise.
    """
    if not isinstance(val, float):
        rospy.logerr('Invalid input type: ' + name + ' must be a float!')
        return None
    if val < low:
        rospy.logwarn(''.join(['Clamping ' + name, ' (' + str(val) + ')',
                               ' to the lower bound: ',  str(low)]))
        return low
    if val > upp:
        rospy.logwarn(''.join(['Clamping ' + name, ' (' + str(val) + ')',
                               ' to the upper bound: ',  str(upp)]))
        return upp
    return val
