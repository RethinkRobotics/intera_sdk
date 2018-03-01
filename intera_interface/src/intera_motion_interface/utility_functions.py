#! /usr/bin/env python

# Copyright (c) 2016-2018, Rethink Robotics Inc.
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
import rospy
from copy import deepcopy


def get_formatted_decimal_string(i, n):
    """
    @param i: index
    @param n: total number of indicies
    @return: str(i) with correct number of padding zeros
    """
    n_digits = len(str(n))
    format_string = '{{:0>{}}}'.format(n_digits)
    return format_string.format(i)


def ensure_path_to_file_exists(raw_file_path):
    """
    This function does two checks:
    1) Expands the ~ in the file path string
    2) Creates any intermediate directories that do not exist
    @param raw_file_path
    @return: file_name = valid path to file
    """
    file_name = os.path.expanduser(raw_file_path)
    file_dir = os.path.dirname(file_name)
    if file_dir and not os.path.exists(file_dir):
        os.makedirs(file_dir)
    return file_name


def is_valid_check_list_for_none(list_data):
    """
    @param list_data: a python list
    @return true iff all elements in the list are not None
    """
    if None in list_data:
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

def int2bool(var):
    """
    Convert integer value/list to bool value/list
    """
    var_out = deepcopy(var)

    if isinstance(var, int):
        var_out = bool(var)
    elif len(var) >= 1:
        for i in range(0, len(var)):
            var_out[i] = bool(var[i])

    return var_out

def bool2int(var):
    """
    Convert bool value/list to int value/list
    """
    var_out = deepcopy(var)

    if isinstance(var, bool):
        var_out = int(var)
    elif len(var) >= 1:
        for i in range(0, len(var)):
            var_out[i] = int(var[i])

    return var_out

def boolToggle(var):
    """
    Toggle bool value/list
    """
    var_out = deepcopy(var)

    if isinstance(var, bool):
        var_out = not var
    elif len(var) >= 1:
        for i in range(0, len(var)):
            var_out[i] = not var[i]

    return var_out