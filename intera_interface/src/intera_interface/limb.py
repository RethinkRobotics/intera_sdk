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

import collections
import warnings

from copy import deepcopy

import rospy

from sensor_msgs.msg import (
    JointState
)
from std_msgs.msg import (
    Float64,
    Header
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

import intera_dataflow

from intera_core_msgs.msg import (
    JointCommand,
    EndpointState,
    EndpointStates,
    CollisionDetectionState,
)
from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
    SolvePositionFK,
    SolvePositionFKRequest
)
import settings
from robot_params import RobotParams


class Limb(object):
    """
    Interface class for a limb on Intera robots.
    """

    # Containers
    Point = collections.namedtuple('Point', ['x', 'y', 'z'])
    Quaternion = collections.namedtuple('Quaternion', ['x', 'y', 'z', 'w'])

    def __init__(self, limb="right", synchronous_pub=False):
        """
        Constructor.

        @type limb: str
        @param limb: limb to interface

        @type synchronous_pub: bool
        @param synchronous_pub: designates the JointCommand Publisher
            as Synchronous if True and Asynchronous if False.

            Synchronous Publishing means that all joint_commands publishing to
            the robot's joints will block until the message has been serialized
            into a buffer and that buffer has been written to the transport
            of every current Subscriber. This yields predicable and consistent
            timing of messages being delivered from this Publisher. However,
            when using this mode, it is possible for a blocking Subscriber to
            prevent the joint_command functions from exiting. Unless you need exact
            JointCommand timing, default to Asynchronous Publishing (False).

            For more information about Synchronous Publishing see:
            http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers#queue_size:_publish.28.29_behavior_and_queuing
        """
        params = RobotParams()
        limb_names = params.get_limb_names()
        if limb not in limb_names:
            rospy.logerr("Cannot detect limb {0} on this robot."
                         " Valid limbs are {1}. Exiting Limb.init().".format(
                                                            limb, limb_names))
            return
        joint_names = params.get_joint_names(limb)
        if not joint_names:
            rospy.logerr("Cannot detect joint names for limb {0} on this "
                         "robot. Exiting Limb.init().".format(limb))
            return
        self.name = limb
        self._joint_angle = dict()
        self._joint_velocity = dict()
        self._joint_effort = dict()
        self._cartesian_pose = dict()
        self._cartesian_velocity = dict()
        self._cartesian_effort = dict()
        self._joint_names = joint_names
        self._collision_state = False
        self._tip_states = None

        ns = '/robot/limb/' + limb + '/'

        self._command_msg = JointCommand()

        self._pub_speed_ratio = rospy.Publisher(
            ns + 'set_speed_ratio',
            Float64,
            latch=True,
            queue_size=10)

        queue_size = None if synchronous_pub else 1
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            self._pub_joint_cmd = rospy.Publisher(
                ns + 'joint_command',
                JointCommand,
                tcp_nodelay=True,
                queue_size=queue_size)

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

        _tip_states_sub = rospy.Subscriber(
            ns + 'tip_states',
            EndpointStates,
            self._on_tip_states,
            queue_size=1,
            tcp_nodelay=True)

        _collision_state_sub = rospy.Subscriber(
            ns + 'collision_detection_state',
            CollisionDetectionState,
            self._on_collision_state,
            queue_size=1,
            tcp_nodelay=True)

        joint_state_topic = 'robot/joint_states'
        _joint_state_sub = rospy.Subscriber(
            joint_state_topic,
            JointState,
            self._on_joint_states,
            queue_size=1,
            tcp_nodelay=True)

        ns_pkn = "ExternalTools/" + limb + "/PositionKinematicsNode/"
        self._iksvc = rospy.ServiceProxy(ns_pkn + 'IKService', SolvePositionIK)
        self._fksvc = rospy.ServiceProxy(ns_pkn + 'FKService', SolvePositionFK)
        rospy.wait_for_service(ns_pkn + 'IKService', 5.0)
        rospy.wait_for_service(ns_pkn + 'FKService', 5.0)

        err_msg = ("%s limb init failed to get current joint_states "
                   "from %s") % (self.name.capitalize(), joint_state_topic)
        intera_dataflow.wait_for(lambda: len(self._joint_angle.keys()) > 0,
                                 timeout_msg=err_msg, timeout=5.0)
        err_msg = ("%s limb init failed to get current endpoint_state "
                   "from %s") % (self.name.capitalize(), ns + 'endpoint_state')
        intera_dataflow.wait_for(lambda: len(self._cartesian_pose.keys()) > 0,
                                 timeout_msg=err_msg, timeout=5.0)
        err_msg = ("%s limb init failed to get current tip_states "
                   "from %s") % (self.name.capitalize(), ns + 'tip_states')
        intera_dataflow.wait_for(lambda: self._tip_states is not None,
                                 timeout_msg=err_msg, timeout=5.0)

    def _on_joint_states(self, msg):
        for idx, name in enumerate(msg.name):
            if name in self._joint_names:
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

    def _on_tip_states(self, msg):
        self._tip_states = deepcopy(msg)

    def _on_collision_state(self, msg):
        if self._collision_state != msg.collision_state:
            self._collision_state = msg.collision_state

    def has_collided(self):
        """
        Return True if the specified limb has experienced a collision.

        @rtype: bool
        @return: True if the arm is in collision, False otherwise.
        """
        return self._collision_state

    def joint_names(self):
        """
        Return the names of the joints for the specified limb.

        @rtype: [str]
        @return: ordered list of joint names from proximal to distal
        (i.e. shoulder to wrist).
        """
        return self._joint_names

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

    def joint_ordered_angles(self):
        """
        Return all joint angles.

        @rtype: [double]
        @return: joint angles (rad) orded by joint_names from proximal to distal
        (i.e. shoulder to wrist).
        """
        return [self._joint_angle[name] for name in self._joint_names]

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

    def tip_state(self, tip_name):
        """
        Return Cartesian endpoint state for a given tip name

        @rtype: EndpointState message
        @return: pose, velocity, effort
        """
        try:
            return deepcopy(self._tip_states.states[self._tip_states.names.index(tip_name)])
        except ValueError:
            return None

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

    def set_joint_position_speed(self, speed=0.3):
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

    def set_joint_trajectory(self, names, positions, velocities, accelerations):
        """
        Commands the joints of this limb to the specified positions using
        the commanded velocities and accelerations to extrapolate between
        commanded positions (prior to the next position being received).

        B{IMPORTANT:} Joint Trajectory control mode allows for commanding
        joint positions, without modification, directly to the JCBs
        (Joint Controller Boards). While this results in more unaffected
        motions, Joint Trajectory control mode bypasses the safety system
        modifications (e.g. collision avoidance).
        Please use with caution.

        @type names: list [str]
        @param names: joint_names list of strings
        @type positions: list [float]
        @param positions: list of positions in radians
        @type velocities: list [float]
        @param velocities: list of velocities in radians/second
        @type accelerations: list [float]
        @param accelerations: list of accelerations in radians/seconds^2
        """
        self._command_msg.names = names
        self._command_msg.position = positions
        self._command_msg.velocity = velocities
        self._command_msg.acceleration = accelerations
        self._command_msg.mode = JointCommand.TRAJECTORY_MODE
        self._command_msg.header.stamp = rospy.Time.now()
        self._pub_joint_cmd.publish(self._command_msg)

    def set_joint_positions(self, positions):
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
        self._command_msg.position = positions.values()
        self._command_msg.mode = JointCommand.POSITION_MODE
        self._command_msg.header.stamp = rospy.Time.now()
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
        self._command_msg.velocity = velocities.values()
        self._command_msg.mode = JointCommand.VELOCITY_MODE
        self._command_msg.header.stamp = rospy.Time.now()
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
        self._command_msg.effort = torques.values()
        self._command_msg.mode = JointCommand.TORQUE_MODE
        self._command_msg.header.stamp = rospy.Time.now()
        self._pub_joint_cmd.publish(self._command_msg)

    def move_to_neutral(self, timeout=15.0, speed=0.3):
        """
        Command the Limb joints to a predefined set of "neutral" joint angles.
        From rosparam named_poses/<limb>/poses/neutral.

        @type timeout: float
        @param timeout: seconds to wait for move to finish [15]
        @type speed: float
        @param speed: ratio of maximum joint speed for execution
                      default= 0.3; range= [0.0-1.0]
        """
        try:
            neutral_pose = rospy.get_param("named_poses/{0}/poses/neutral".format(self.name))
        except KeyError:
            rospy.logerr(("Get neutral pose failed, arm: \"{0}\".").format(self.name))
            return
        angles = dict(zip(self.joint_names(), neutral_pose))
        self.set_joint_position_speed(speed)
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

        def genf(joint, angle):
            def joint_diff():
                return abs(angle - self._joint_angle[joint])
            return joint_diff

        diffs = [genf(j, a) for j, a in positions.items() if
                 j in self._joint_angle]
        fail_msg = "{0} limb failed to reach commanded joint positions.".format(
                                                      self.name.capitalize())
        def test_collision():
            if self.has_collided():
                rospy.logerr(' '.join(["Collision detected.", fail_msg]))
                return True
            return False
        self.set_joint_positions(positions)
        intera_dataflow.wait_for(
            test=lambda: test_collision() or \
                         (callable(test) and test() == True) or \
                         (all(diff() < threshold for diff in diffs)),
            timeout=timeout,
            timeout_msg=fail_msg,
            rate=100,
            raise_on_error=False,
            body=lambda: self.set_joint_positions(positions)
            )

    def ik_request(self, pose,
                   end_point='right_hand', joint_seed=None,
                   nullspace_goal=None, nullspace_gain=0.4,
                   allow_collision=False):
        """
        Inverse Kinematics request sent to IK Service

        @type pose: geometry_msgs.Pose
        @param pose: Cartesian pose of the end point
        @type end_point: string
        @param end_point: name of the end point (should be in URDF)
        @type joint_seed: dict({str:float})
        @param joint_seed: the joint angles for the initial IK guess (optional)
        @type nullspace_goal: dict({str:float})
        @param nullspace_goal: desired joints, or subset of joints, to bias the solver (optional)
        @type nullspace_gain: double
        @param nullspace_gain: gain used to bias toward the nullspace goal [0.0, 1.0] (optional)
        @type allow_collision: bool
        @param allow_collision: does not check if Ik solution is in collision
        @rtype: dict({str:float})
        @return: valid joint positions if exists.  False if no solution is found.
        """
        if not isinstance(pose, Pose):
            rospy.logerr('pose is not of type geometry_msgs.msgs.Pose')
            return False

        # Add desired pose for inverse kinematics
        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        ikreq.tip_names.append(end_point)

        # The joint seed is where the IK position solver starts its optimization
        if joint_seed is not None:
            ikreq.seed_mode = ikreq.SEED_USER
            seed = JointState()
            seed.name = joint_seed.keys()
            seed.position = joint_seed.values()
            ikreq.seed_angles.append(seed)

        # Once the primary IK task is solved, the solver will then try to bias the
        # the joint angles toward the goal joint configuration. The null space is
        # the extra degrees of freedom the joints can move without affecting the
        # primary IK task.
        if nullspace_goal is not None:
          ikreq.use_nullspace_goal.append(True)
          # The nullspace goal can either be the full set or subset of joint angles
          goal = JointState()
          goal.names = nullspace_goal.keys()
          goal.position = nullspace_goal.values()
          ikreq.nullspace_goal.append(goal)
          # The gain used to bias toward the nullspace goal. Must be [0.0, 1.0]
          # If empty, the default gain of 0.4 will be used
          ikreq.nullspace_gain.append(nullspace_gain)

        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("IK Service call failed: %s" % (e,))
            return False
        limb_joints = {}
        # Check if result valid, and type of seed ultimately used to get solution
        if (resp.result_type[0] > 0
                or (allow_collision and resp.result_type[0] == resp.IK_IN_COLLISION)):
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints

    def fk_request(self, joint_angles,
                   end_point='right_hand'):
        """
        Forward Kinematics request sent to FK Service

        @type joint_angles: dict({str:float})
        @param joint_angles: the arm's joint positions
        @type end_point: string
        @param end_point: name of the end point (should be in URDF)
        @return: Forward Kinematics response from FK service
        """
        fkreq = SolvePositionFKRequest()
        # Add desired pose for forward kinematics
        joints = JointState()
        joints.name = joint_angles.keys()
        joints.position = joint_angles.values()
        fkreq.configuration.append(joints)
        # Request forward kinematics from base to end_point
        fkreq.tip_names.append(end_point)
        try:
            resp = self._fksvc(fkreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("FK Service call failed: %s" % (e,))
            return False
        return resp

    def joint_angles_to_cartesian_pose(self, joint_angles,
                                       end_point='right_hand'):
        """
        Convert joint angles to a Cartesian Pose. Calls FK service

        @type joint_angles: dict({str:float})
        @param joint_angles: the arm's joint positions
        @type end_point: string
        @param end_point: name of the end point (should be in URDF)
        @rtype: geometry_msgs.Pose
        @return: cartesian pose associated with joint state
        """
        resp = self.fk_request(joint_angles, end_point)

        if resp is None:
            rospy.logerr("Failed to get cartesian pose - check FK server.")
            return None
        if not resp.isValid[0]:
            rospy.loginfo("INVALID JOINTS - No Cartesian Solution Found.")
            return None
        return resp.pose_stamp[0].pose

    def fk_request_in_collision(self, joint_angles,
                                end_point='right_hand'):
        """
        Checks if the arm will be in collision with these joint angles.
        Calls FK service

        @type joint_angles: dict({str:float})
        @param joint_angles: the arm's joint positions
        @type end_point: string
        @param end_point: name of the end point (should be in URDF)
        @type pose: geometry_msgs.Pose
        @rtype: bool
        @return: True if in collision
        """
        resp = self.fk_request(joint_angles, end_point)
        if resp is None:
            rospy.logerr("Failed to get cartesian pose - check FK server.")
            return True
        if not resp.isValid[0]:
            rospy.loginfo("INVALID JOINTS - No Cartesian Solution Found.")
            return True
        return resp.inCollision[0]
