#! /usr/bin/env python
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

"""
Intera RSDK Forward Kinematics Example
"""
import rospy
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

from intera_core_msgs.srv import (
    SolvePositionFK,
    SolvePositionFKRequest,
)

def fk_service_client(limb = "right"):
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/FKService"
    fksvc = rospy.ServiceProxy(ns, SolvePositionFK)
    fkreq = SolvePositionFKRequest()
    joints = JointState()
    joints.name = ['right_j0', 'right_j1', 'right_j2', 'right_j3',
                   'right_j4', 'right_j5', 'right_j6']
    joints.position = [0.763331, 0.415979, -1.728629, 1.482985,
                       -1.135621, -1.674347, -0.496337]
    # Add desired pose for forward kinematics
    fkreq.configuration.append(joints)
    # Request forward kinematics from base to "right_hand" link
    fkreq.tip_names.append('right_hand')

    try:
        rospy.wait_for_service(ns, 5.0)
        resp = fksvc(fkreq)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr("Service call failed: %s" % (e,))
        return False

    # Check if result valid
    if (resp.isValid[0]):
        rospy.loginfo("SUCCESS - Valid Cartesian Solution Found")
        rospy.loginfo("\nFK Cartesian Solution:\n")
        rospy.loginfo("------------------")
        rospy.loginfo("Response Message:\n%s", resp)
    else:
        rospy.logerr("INVALID JOINTS - No Cartesian Solution Found.")
        return False

    return True


def main():
    """RSDK Forward Kinematics Example

    A simple example of using the Rethink Forward Kinematics
    Service which returns the Cartesian Pose based on the input joint angles.

    Run this example, the example will use the default limb
    and call the Service with a sample Joint Position,
    pre-defined in the example code, printing the
    response of whether a valid Cartesian solution was found,
    and if so, the corresponding Cartesian pose.
    """
    rospy.init_node("rsdk_fk_service_client")

    if fk_service_client():
        rospy.loginfo("Simple FK call passed!")
    else:
        rospy.logerr("Simple FK call FAILED")


if __name__ == '__main__':
    main()
