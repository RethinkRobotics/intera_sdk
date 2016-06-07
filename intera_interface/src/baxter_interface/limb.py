# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import collections

from copy import deepcopy

import rospy

from sensor_msgs.msg import (
    JointState
)
from std_msgs.msg import (
    Float64,
)

import baxter_dataflow

from baxter_core_msgs.msg import (
    JointCommand,
    EndpointState,
)
from baxter_interface import settings


class Limb(object):
    """
    Interface class for a limb on the Baxter robot.
    """

    # Containers
    Point = collections.namedtuple('Point', ['x', 'y', 'z'])
    Quaternion = collections.namedtuple('Quaternion', ['x', 'y', 'z', 'w'])

    def __init__(self, limb):
        """
        Constructor.

        @type limb: str
        @param limb: limb to interface
        """
        self.name = limb
        self._joint_angle = dict()
        self._joint_velocity = dict()
        self._joint_effort = dict()
        self._cartesian_pose = dict()
        self._cartesian_velocity = dict()
        self._cartesian_effort = dict()

        self._joint_names = {
            'left': ['left_s0', 'left_s1', 'left_e0', 'left_e1',
                     'left_w0', 'left_w1', 'left_w2'],
            'right': ['right_s0', 'right_s1', 'right_e0', 'right_e1',
                      'right_w0', 'right_w1', 'right_w2']
            }

        ns = '/robot/limb/' + limb + '/'

        self._command_msg = JointCommand()

        self._pub_speed_ratio = rospy.Publisher(
            ns + 'set_speed_ratio',
            Float64,
            latch=True,
            queue_size=10)

        self._pub_joint_cmd = rospy.Publisher(
            ns + 'joint_command',
            JointCommand,
            tcp_nodelay=True,
            queue_size=1)

        self._pub_joint_cmd_timeout = rospy.Publisher(
            ns + 'joint_command_timeout',
            Float64,
            latch=True,
            queue_size=10)

        _cartesian_state_sub = rospy.Subscriber(
            ns + 'endpoint_state',
            EndpointState,
            self._on_endpoint_states,
            queue_size=1,
            tcp_nodelay=True)

        joint_state_topic = 'robot/joint_states'
        _joint_state_sub = rospy.Subscriber(
            joint_state_topic,
            JointState,
            self._on_joint_states,
            queue_size=1,
            tcp_nodelay=True)

        err_msg = ("%s limb init failed to get current joint_states "
                   "from %s") % (self.name.capitalize(), joint_state_topic)
        baxter_dataflow.wait_for(lambda: len(self._joint_angle.keys()) > 0,
                                 timeout_msg=err_msg)
        err_msg = ("%s limb init failed to get current endpoint_state "
                   "from %s") % (self.name.capitalize(), ns + 'endpoint_state')
        baxter_dataflow.wait_for(lambda: len(self._cartesian_pose.keys()) > 0,
                                 timeout_msg=err_msg)

    def _on_joint_states(self, msg):
        for idx, name in enumerate(msg.name):
            if name in self._joint_names[self.name]:
                self._joint_angle[name] = msg.position[idx]
                self._joint_velocity[name] = msg.velocity[idx]
                self._joint_effort[name] = msg.effort[idx]

    def _on_endpoint_states(self, msg):
        # Comments in this private method are for documentation purposes.
        # _pose = {'position': (x, y, z), 'orientation': (x, y, z, w)}
        self._cartesian_pose = {
            'position': self.Point(
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ),
            'orientation': self.Quaternion(
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ),
        }
        # _twist = {'linear': (x, y, z), 'angular': (x, y, z)}
        self._cartesian_velocity = {
            'linear': self.Point(
                msg.twist.linear.x,
                msg.twist.linear.y,
                msg.twist.linear.z,
            ),
            'angular': self.Point(
                msg.twist.angular.x,
                msg.twist.angular.y,
                msg.twist.angular.z,
            ),
        }
        # _wrench = {'force': (x, y, z), 'torque': (x, y, z)}
        self._cartesian_effort = {
            'force': self.Point(
                msg.wrench.force.x,
                msg.wrench.force.y,
                msg.wrench.force.z,
            ),
            'torque': self.Point(
                msg.wrench.torque.x,
                msg.wrench.torque.y,
                msg.wrench.torque.z,
            ),
        }

    def joint_names(self):
        """
        Return the names of the joints for the specified limb.

        @rtype: [str]
        @return: ordered list of joint names from proximal to distal
        (i.e. shoulder to wrist).
        """
        return self._joint_names[self.name]

    def joint_angle(self, joint):
        """
        Return the requested joint angle.

        @type joint: str
        @param joint: name of a joint
        @rtype: float
        @return: angle in radians of individual joint
        """
        return self._joint_angle[joint]

    def joint_angles(self):
        """
        Return all joint angles.

        @rtype: dict({str:float})
        @return: unordered dict of joint name Keys to angle (rad) Values
        """
        return deepcopy(self._joint_angle)

    def joint_velocity(self, joint):
        """
        Return the requested joint velocity.

        @type joint: str
        @param joint: name of a joint
        @rtype: float
        @return: velocity in radians/s of individual joint
        """
        return self._joint_velocity[joint]

    def joint_velocities(self):
        """
        Return all joint velocities.

        @rtype: dict({str:float})
        @return: unordered dict of joint name Keys to velocity (rad/s) Values
        """
        return deepcopy(self._joint_velocity)

    def joint_effort(self, joint):
        """
        Return the requested joint effort.

        @type joint: str
        @param joint: name of a joint
        @rtype: float
        @return: effort in Nm of individual joint
        """
        return self._joint_effort[joint]

    def joint_efforts(self):
        """
        Return all joint efforts.

        @rtype: dict({str:float})
        @return: unordered dict of joint name Keys to effort (Nm) Values
        """
        return deepcopy(self._joint_effort)

    def endpoint_pose(self):
        """
        Return Cartesian endpoint pose {position, orientation}.

        @rtype: dict({str:L{Limb.Point},str:L{Limb.Quaternion}})
        @return: position and orientation as named tuples in a dict

        C{pose = {'position': (x, y, z), 'orientation': (x, y, z, w)}}

          - 'position': Cartesian coordinates x,y,z in
                        namedtuple L{Limb.Point}
          - 'orientation': quaternion x,y,z,w in named tuple
                           L{Limb.Quaternion}
        """
        return deepcopy(self._cartesian_pose)

    def endpoint_velocity(self):
        """
        Return Cartesian endpoint twist {linear, angular}.

        @rtype: dict({str:L{Limb.Point},str:L{Limb.Point}})
        @return: linear and angular velocities as named tuples in a dict

        C{twist = {'linear': (x, y, z), 'angular': (x, y, z)}}

          - 'linear': Cartesian velocity in x,y,z directions in
                      namedtuple L{Limb.Point}
          - 'angular': Angular velocity around x,y,z axes in named tuple
                       L{Limb.Point}
        """
        return deepcopy(self._cartesian_velocity)

    def endpoint_effort(self):
        """
        Return Cartesian endpoint wrench {force, torque}.

        @rtype: dict({str:L{Limb.Point},str:L{Limb.Point}})
        @return: force and torque at endpoint as named tuples in a dict

        C{wrench = {'force': (x, y, z), 'torque': (x, y, z)}}

          - 'force': Cartesian force on x,y,z axes in
                     namedtuple L{Limb.Point}
          - 'torque': Torque around x,y,z axes in named tuple
                      L{Limb.Point}
        """
        return deepcopy(self._cartesian_effort)

    def set_command_timeout(self, timeout):
        """
        Set the timeout in seconds for the joint controller

        @type timeout: float
        @param timeout: timeout in seconds
        """
        self._pub_joint_cmd_timeout.publish(Float64(timeout))

    def exit_control_mode(self, timeout=0.2):
        """
        Clean exit from advanced control modes (joint torque or velocity).

        Resets control to joint position mode with current positions.

        @type timeout: float
        @param timeout: control timeout in seconds [0.2]
        """
        self.set_command_timeout(timeout)
        self.set_joint_positions(self.joint_angles())

    def set_joint_position_speed(self, speed):
        """
        Set ratio of max joint speed to use during joint position moves.

        Set the proportion of maximum controllable velocity to use
        during joint position control execution. The default ratio
        is `0.3`, and can be set anywhere from [0.0-1.0] (clipped).
        Once set, a speed ratio will persist until a new execution
        speed is set.

        @type speed: float
        @param speed: ratio of maximum joint speed for execution
                      default= 0.3; range= [0.0-1.0]
        """
        self._pub_speed_ratio.publish(Float64(speed))

    def set_joint_positions(self, positions, raw=False):
        """
        Commands the joints of this limb to the specified positions.

        B{IMPORTANT:} 'raw' joint position control mode allows for commanding
        joint positions, without modification, directly to the JCBs
        (Joint Controller Boards). While this results in more unaffected
        motions, 'raw' joint position control mode bypasses the safety system
        modifications (e.g. collision avoidance).
        Please use with caution.

        @type positions: dict({str:float})
        @param positions: joint_name:angle command
        @type raw: bool
        @param raw: advanced, direct position control mode
        """
        self._command_msg.names = positions.keys()
        self._command_msg.command = positions.values()
        if raw:
            self._command_msg.mode = JointCommand.RAW_POSITION_MODE
        else:
            self._command_msg.mode = JointCommand.POSITION_MODE
        self._pub_joint_cmd.publish(self._command_msg)

    def set_joint_velocities(self, velocities):
        """
        Commands the joints of this limb to the specified velocities.

        B{IMPORTANT}: set_joint_velocities must be commanded at a rate great
        than the timeout specified by set_command_timeout. If the timeout is
        exceeded before a new set_joint_velocities command is received, the
        controller will switch modes back to position mode for safety purposes.

        @type velocities: dict({str:float})
        @param velocities: joint_name:velocity command
        """
        self._command_msg.names = velocities.keys()
        self._command_msg.command = velocities.values()
        self._command_msg.mode = JointCommand.VELOCITY_MODE
        self._pub_joint_cmd.publish(self._command_msg)

    def set_joint_torques(self, torques):
        """
        Commands the joints of this limb to the specified torques.

        B{IMPORTANT}: set_joint_torques must be commanded at a rate great than
        the timeout specified by set_command_timeout. If the timeout is
        exceeded before a new set_joint_torques command is received, the
        controller will switch modes back to position mode for safety purposes.

        @type torques: dict({str:float})
        @param torques: joint_name:torque command
        """
        self._command_msg.names = torques.keys()
        self._command_msg.command = torques.values()
        self._command_msg.mode = JointCommand.TORQUE_MODE
        self._pub_joint_cmd.publish(self._command_msg)

    def move_to_neutral(self, timeout=15.0):
        """
        Command the joints to the center of their joint ranges

        Neutral is defined as::
          ['*_s0', '*_s1', '*_e0', '*_e1', '*_w0', '*_w1', '*_w2']
          [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]

        @type timeout: float
        @param timeout: seconds to wait for move to finish [15]
        """
        angles = dict(zip(self.joint_names(),
                          [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]))
        return self.move_to_joint_positions(angles, timeout)

    def move_to_joint_positions(self, positions, timeout=15.0,
                                threshold=settings.JOINT_ANGLE_TOLERANCE,
                                test=None):
        """
        (Blocking) Commands the limb to the provided positions.

        Waits until the reported joint state matches that specified.

        This function uses a low-pass filter to smooth the movement.

        @type positions: dict({str:float})
        @param positions: joint_name:angle command
        @type timeout: float
        @param timeout: seconds to wait for move to finish [15]
        @type threshold: float
        @param threshold: position threshold in radians across each joint when
        move is considered successful [0.008726646]
        @param test: optional function returning True if motion must be aborted
        """
        cmd = self.joint_angles()

        def filtered_cmd():
            # First Order Filter - 0.2 Hz Cutoff
            for joint in positions.keys():
                cmd[joint] = 0.012488 * positions[joint] + 0.98751 * cmd[joint]
            return cmd

        def genf(joint, angle):
            def joint_diff():
                return abs(angle - self._joint_angle[joint])
            return joint_diff

        diffs = [genf(j, a) for j, a in positions.items() if
                 j in self._joint_angle]

        self.set_joint_positions(filtered_cmd())
        baxter_dataflow.wait_for(
            test=lambda: callable(test) and test() == True or \
                         (all(diff() < threshold for diff in diffs)),
            timeout=timeout,
            timeout_msg=("%s limb failed to reach commanded joint positions" %
                         (self.name.capitalize(),)),
            rate=100,
            raise_on_error=False,
            body=lambda: self.set_joint_positions(filtered_cmd())
            )
