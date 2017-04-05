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


def import_all_waypoint_sequences(directory_name, joint_name_check=None,
                                  sort_files=True, make_periodic=False):
    if not os.path.isdir(directory_name):
        rospy.logerr('Failed to import waypoint sequences: %s',
                     directory_name + ' is not a directory!')
        return None
    path_names = []
    for root, dirs, files in os.walk(directory_name, topdown=False):
        for name in files:
            path_names.append(os.path.join(root, name))
    if sort_files:
        path_names.sort()
    waypoint_sequence_list = []
    for name in path_names:
        wpt_seq = import_waypoint_sequence(name, make_periodic=make_periodic)
        if wpt_seq is not None:
            waypoint_sequence_list.append(wpt_seq)
    return waypoint_sequence_list


def import_waypoint_sequence(file_name, joint_name_check=None,
                             make_periodic=False):
    """
    Import a waypoint sequence from a csv file
    @param file_name: absolute path to a csv file
    @param joint_name_check: a list of joint names to check against the file
                             (skip check if joint_name_check is None)
    """
    with open(file_name, 'rb') as csvfile:
        csvreader = csv.reader(csvfile)
        joint_names = None
        waypoint_sequence = []
        count = 0
        for row in csvreader:
            if joint_names is None:  # True only on the first call
                joint_names = deepcopy(row)
                if joint_name_check is not None:
                    if not joint_names == joint_name_check:
                        rospy.logerr("Trajectory has invalid joint names!")
                        return None
            else:  # all rows except for the first
                waypoint = []
                for data in row:
                    waypoint.append(float(data))
                waypoint_sequence.append(waypoint)
                count += 1
    if count == 0:
        name = os.path.basename(file_name)
        rospy.logerr('csv file: ' + name + ' contains no data!')
        return None
    if make_periodic:  # add the first waypoint to the end
        waypoint_sequence.append(waypoint_sequence[0])
    return waypoint_sequence


def write_waypoint_sequence(file_name, wpt_seq, joint_names):
    """
    @param file_name: file name to write waypoint sequence to
    @param joint_names: joint names for first line of the file
    """
    file_name = ensure_path_to_file_exists(file_name)
    waypoint_data = []
    waypoint_data.append(joint_names)
    for wpt in wpt_seq:
        waypoint_data.append(wpt)
    with open(file_name, "wb") as f:
        writer = csv.writer(f)
        writer.writerows(waypoint_data)


# TODO: This should eventually use the SDK IK interface
def cartesian_pose_to_joint_angles(pose, joint_seed,
                                   end_point='right_hand',
                                   joint_names=['right_j0', 'right_j1',
                                                'right_j2', 'right_j3',
                                                'right_j4', 'right_j5',
                                                'right_j6']):


    ns = "ExternalTools/right/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()

    # Add desired pose for inverse kinematics
    ikreq.pose_stamp.append(pose)
    ikreq.tip_names.append(end_point)

    # The joint seed is where the IK position solver starts its optimization
    ikreq.seed_mode = ikreq.SEED_USER
    seed = JointState()
    seed.name = joint_names
    seed.position = joint_seed
    ikreq.seed_angles.append(seed)

    # Once the primary IK task is solved, the solver will then try to bias the
    # the joint angles toward the goal joint configuration. The null space is
    # the extra degrees of freedom the joints can move without affecting the
    # primary IK task.
    ikreq.use_nullspace_goal.append(True)
    # The nullspace goal can either be the full set or subset of joint angles
    goal = JointState()
    goal.name = joint_names
    goal.position = joint_seed
    ikreq.nullspace_goal.append(goal)
    # The gain used to bias toward the nullspace goal. Must be [0.0, 1.0]
    # If empty, the default gain of 0.4 will be used
    ikreq.nullspace_gain.append(0.4)

    # TODO: this is not a great implementation (too slow). Use SDK instead.
    for i_try in range(0, 5):
        try:
            rospy.wait_for_service(ns, 2.0)
            resp = iksvc(ikreq)
            break;
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logwarn("Failed to get joint configuration (try %d/%d)",
                i_try + 1, 5)
            resp = None
        if rospy.is_shutdown():
            rospy.logwarn('ROS node was shut down!')
            break
    if resp is None:
        rospy.logerr("Failed to get joint configuration - check IK server.")
        return None

    # Check if result valid, and type of seed ultimately used to get solution
    if (resp.result_type[0] > 0):
        return resp.joints[0].position
    else:
        rospy.loginfo("INVALID POSE - No Valid Joint Solution Found.")
        return None


# TODO: Move to SDK FK interface
def joint_angles_to_cartesian_pose(joint_angles,
                                   joint_names=['right_j0', 'right_j1',
                                                'right_j2', 'right_j3',
                                                'right_j4', 'right_j5',
                                                'right_j6'],
                                   end_point='right_hand'):
    """
    @return: cartesian pose associated with joint state
    """
    resp = get_fk_response(joint_angles, joint_names, end_point)

    if resp is None:
        rospy.logerr("Failed to get cartesian pose - check FK server.")
        return None
    if not resp.isValid[0]:
        rospy.loginfo("INVALID JOINTS - No Cartesian Solution Found.")
        return None
    return resp.pose_stamp[0]

def in_collision(angles,
                 joint_names=['right_j0', 'right_j1',
                              'right_j2', 'right_j3',
                              'right_j4', 'right_j5',
                              'right_j6'],
                 end_point='right_hand'):
    """
    @return: cartesian pose associated with joint state
    """
    resp = get_fk_response(angles, joint_names, end_point)
    if resp is None:
        rospy.logerr("Failed to get cartesian pose - check FK server.")
        return True
    if not resp.isValid[0]:
        rospy.loginfo("INVALID JOINTS - No Cartesian Solution Found.")
        return True
    return resp.inCollision[0]

def get_fk_response(angles,
                    joint_names=['right_j0', 'right_j1',
                                 'right_j2', 'right_j3',
                                 'right_j4', 'right_j5',
                                 'right_j6'],
                    end_point='right_hand'):
    """
    @return: forward kinematics response from FK service
    """
    # TODO - cleanup and check inputs
    joints = JointState()
    joints.name = joint_names
    joints.position = angles
    ns = "ExternalTools/right/PositionKinematicsNode/FKService"
    fksvc = rospy.ServiceProxy(ns, SolvePositionFK)
    fkreq = SolvePositionFKRequest()
    # Add desired pose for forward kinematics
    fkreq.configuration.append(joints)
    # Request forward kinematics from base to "right_hand" link
    fkreq.tip_names.append(end_point)
    for i_try in range(0, 5):
        try:
            rospy.wait_for_service(ns, 2.0)
            resp = fksvc(fkreq)
            break;
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logwarn("Failed to get cartesian pose (try %d/%d)",
                          i_try + 1, 4)
            resp = None
        if rospy.is_shutdown():
            rospy.logwarn('ROS node was shut down!')
            resp = None
            break
    return resp

def is_valid_check_list_for_none(list):
    for data in list:
        if not data:
            rospy.logwarn("This list contains at least one None value!")
            return False
    return True


# This file is from intera_sdk/intera_interface/src/intera_dataflow/wai_for.py
def wait_for(test, timeout=1.0, raise_on_error=True, rate=100,
             timeout_msg="timeout expired", body=None):
    """
    Waits until some condition evaluates to true.
    @param test: zero param function to be evaluated
    @param timeout: max amount of time to wait. negative/inf for indefinitely
    @param raise_on_error: raise or just return False
    @param rate: the rate at which to check
    @param timout_msg: message to supply to the timeout exception
    @param body: optional function to execute while waiting
    """
    end_time = rospy.get_time() + timeout
    rate = rospy.Rate(rate)
    notimeout = (timeout < 0.0) or timeout == float("inf")
    while not test():
        if rospy.is_shutdown():
            if raise_on_error:
                raise OSError(errno.ESHUTDOWN, "ROS Shutdown")
            return False
        elif (not notimeout) and (rospy.get_time() >= end_time):
            if raise_on_error:
                raise OSError(errno.ETIMEDOUT, timeout_msg)
            return False
        if callable(body):
            body()
        rate.sleep()
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
