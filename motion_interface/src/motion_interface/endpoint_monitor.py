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
import math
import yaml
import csv
from intera_core_msgs.msg import EndpointState
from copy import deepcopy
from threading import RLock
from utility_functions import ensure_path_to_file_exists
import numpy as np
import matplotlib.pyplot as plt

class EndpointMonitor(object):

    def __init__(self):
        """
        Creates an endpoint monitor. This class is used to accumulate the
        tracking error data and then compute statistics on it.
        """
        self._data = dict()

        self._lock = RLock()

        self.reset()  # populate empty data structure

        self._subscriber = rospy.Subscriber(
            '/robot/limb/right/endpoint_state',
            EndpointState, self._subscriber_callback)

        self._call_back = None

    def _subscriber_callback(self, msg):
        with self._lock:
            if self._is_active:
                self._is_up_to_date = False
                if len(self._data.keys())== 0:
                    self._on_first_call(msg)
                else:
                    self._on_regular_call(msg)

    def start(self, start_time = None):
        self._is_active = True
        if start_time is not None:
            self._start_time = start_time

    def pause(self):
        self._is_active = False

    def reset(self):
        self.pause()
        self._is_active = False
        self._data = dict()  # data to report back to user
        self._joint_names = []
        self._start_time = None

    def is_recording(self):
        return self._is_active

    def set_call_back(self, call_back):
        """
        @param call_back: a user-defined call back that accepts
                          a dictionary that maps joint name to value
        """
        self._call_back = call_back

    def _on_regular_call(self, msg):
        time = msg.header.stamp - self._start_time
        self._data['time'].append(time.to_sec())
        self._data['pose_lin_x'].append(msg.pose.position.x)
        self._data['pose_lin_y'].append(msg.pose.position.y)
        self._data['pose_lin_z'].append(msg.pose.position.z)
        self._data['pose_ang_x'].append(msg.pose.orientation.x)
        self._data['pose_ang_y'].append(msg.pose.orientation.y)
        self._data['pose_ang_z'].append(msg.pose.orientation.z)
        self._data['pose_ang_w'].append(msg.pose.orientation.w)
        self._data['twist_lin_x'].append(msg.twist.linear.x)
        self._data['twist_lin_y'].append(msg.twist.linear.y)
        self._data['twist_lin_z'].append(msg.twist.linear.z)
        self._data['twist_ang_x'].append(msg.twist.angular.x)
        self._data['twist_ang_y'].append(msg.twist.angular.y)
        self._data['twist_ang_z'].append(msg.twist.angular.z)
        self._data['wrench_lin_x'].append(msg.wrench.force.x)
        self._data['wrench_lin_y'].append(msg.wrench.force.y)
        self._data['wrench_lin_z'].append(msg.wrench.force.z)
        self._data['wrench_ang_x'].append(msg.wrench.torque.x)
        self._data['wrench_ang_y'].append(msg.wrench.torque.y)
        self._data['wrench_ang_z'].append(msg.wrench.torque.z)

        if self._call_back is not None:
            self._call_back(deepcopy(msg))

    def _on_first_call(self, msg):
        # Timing Data:
        if self._start_time is None:
            self._start_time = msg.header.stamp

        self._pose_lin_names = ['pose_lin_x', 'pose_lin_y', 'pose_lin_z']
        self._pose_ang_names = ['pose_ang_x', 'pose_ang_y', 'pose_ang_z', 'pose_ang_w']
        self._twist_lin_names = ['twist_lin_x', 'twist_lin_y', 'twist_lin_z']
        self._twist_ang_names = ['twist_ang_x', 'twist_ang_y', 'twist_ang_z']
        self._wrench_lin_names = ['wrench_lin_x', 'wrench_lin_y', 'wrench_lin_z']
        self._wrench_ang_names = ['wrench_ang_x', 'wrench_ang_y', 'wrench_ang_z']

        self._data_keys = ['time']
        self._data_keys.extend(self._pose_lin_names)
        self._data_keys.extend(self._pose_ang_names)
        self._data_keys.extend(self._twist_lin_names)
        self._data_keys.extend(self._twist_ang_names)
        self._data_keys.extend(self._wrench_lin_names)
        self._data_keys.extend(self._wrench_ang_names)

        for name in self._data_keys:
            self._data[name] = []

        # Now update the data:
        self._on_regular_call(msg)

    def get_data(self):
        with self._lock:
            return deepcopy(self._data)

    def to_yaml(self, file_name):
        with self._lock:
            file_name = ensure_path_to_file_exists(file_name)
            with open(file_name, "w") as outfile:
                yaml.dump(self._data, outfile, default_flow_style=False)

    def to_csv(self, file_name):
        with self._lock:
            file_name = ensure_path_to_file_exists(file_name)

            if not any(self._data):
                rospy.logwarn('cannot write to csv:  there is no data!')
                return

            title_row = self._data_keys
            csv_data = []
            csv_data.append(title_row)
            for i in range(0, len(self._data['time'])):
                data_row = []
                for key in self._data_keys:
                    data_row.append(self._data[key][i])
                csv_data.append(deepcopy(data_row))

            with open(file_name, "w") as outfile:
                writer = csv.writer(outfile)
                writer.writerows(csv_data)

    def plot(self, file_name=None, width=24, height=13.5, dpi=300):
        """
        @param file_name:  if ommitted (or None) then display plot. OTHERWISE
                           save plot as svg to this file name.
        """

        if not any(self._data):
            rospy.logwarn('cannot plot:  there is no data!')
            return

        plt.clf()
        fig, axarr = plt.subplots(3, 2, sharex=True)
        pose_lin_ax = axarr[0,0]
        pose_ang_ax = axarr[0,1]
        twist_lin_ax = axarr[1,0]
        twist_ang_ax = axarr[1,1]
        wrench_lin_ax = axarr[2,0]
        wrench_ang_ax = axarr[2,1]

        time = self._data['time']

        for name in self._pose_lin_names:
            pose_lin_ax.plot(time, self._data[name], label=name)
        for name in self._twist_lin_names:
            twist_lin_ax.plot(time, self._data[name], label=name)
        for name in self._wrench_lin_names:
            wrench_lin_ax.plot(time, self._data[name], label=name)
        for name in self._pose_ang_names:
            pose_ang_ax.plot(time, self._data[name], label=name)
        for name in self._twist_ang_names:
            twist_ang_ax.plot(time, self._data[name], label=name)
        for name in self._wrench_ang_names:
            wrench_ang_ax.plot(time, self._data[name], label=name)

        for ax in [pose_lin_ax, pose_ang_ax, twist_lin_ax,
                   twist_ang_ax, wrench_lin_ax, wrench_ang_ax]:
            ax.minorticks_on()
            ax.grid(b=True, which='major', color='k', linestyle='-')
            ax.grid(b=True, which='minor', color='k', linestyle='-', alpha=0.2)
            ax.set_xlabel('time (sec)')
            ax.legend()

        pose_lin_ax.set_ylabel('position (m)')
        pose_ang_ax.set_ylabel('orientation (quaternian)')
        twist_lin_ax.set_ylabel('twist - linear (m/s)')
        twist_ang_ax.set_ylabel('twist - angular (rad/s)')
        wrench_lin_ax.set_ylabel('wrench - linear (N)')
        wrench_ang_ax.set_ylabel('wrench - angular (Nm)')


        if file_name is None:
            fig.show()
        else:
            fig.set_size_inches(width, height)
            fig.savefig(file_name, dpi=dpi)
            plt.close()
