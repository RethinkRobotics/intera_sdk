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
from sensor_msgs.msg import JointState
from copy import deepcopy
from threading import RLock
from utility_functions import ensure_path_to_file_exists
import numpy as np
import matplotlib.pyplot as plt

class JointStateMonitor(object):

    def __init__(self):
        """
        Creates an joint state monitor. This class is used to accumulate the
        tracking error data and then compute statistics on it.
        """
        self._lock = RLock()
        self.reset()  # populate empty data structure

        self._subscriber = rospy.Subscriber(
            '/robot/joint_states',
            JointState, self._subscriber_callback)

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
        self._data = dict()
        self._start_time = None

        self._joint_names = []
        for i in range(0,7):
            self._joint_names.append('J' + str(i));

        self._idx_map = dict()
        for i in range(0,len(self._joint_names)):
            self._idx_map[self._joint_names[i]] = i + 1

        self._pos_names = []
        for name in self._joint_names:
            self._pos_names.append('pos_' + name)

        self._vel_names = []
        for name in self._joint_names:
            self._vel_names.append('vel_' + name)

        self._eff_names = []
        for name in self._joint_names:
            self._eff_names.append('eff_' + name)

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
        for name in self._joint_names:
            self._data['pos_' + name].append(msg.position[self._idx_map[name]])
            self._data['vel_' + name].append(msg.velocity[self._idx_map[name]])
            self._data['eff_' + name].append(msg.effort[self._idx_map[name]])

        if self._call_back is not None:
            self._call_back(deepcopy(msg))

    def _on_first_call(self, msg):
        # Timing Data:
        if self._start_time is None:
            self._start_time = msg.header.stamp

        # Make sure that out names line up:
        for i in range(0,7):
            if msg.name[self._idx_map['J' + str(i)]] != 'right_j' + str(i):
                rospy.logerr('Indexing error in joint state monitor!')

        # Initialize all data - make sure names are in a good order for csv
        self._data_keys = ['time']
        self._data_keys.extend(self._pos_names)
        self._data_keys.extend(self._vel_names)
        self._data_keys.extend(self._eff_names)
        for key in self._data_keys:
            self._data[key] = []

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
        fig, axarr = plt.subplots(3, sharex=True)
        pos_ax = axarr[0]
        vel_ax = axarr[1]
        eff_ax = axarr[2]

        time = self._data['time']

        for name in self._pos_names:
            pos_ax.plot(time, self._data[name], label=name)
        for name in self._vel_names:
            vel_ax.plot(time, self._data[name], label=name)
        for name in self._eff_names:
            eff_ax.plot(time, self._data[name], label=name)

        for ax in axarr:
            ax.minorticks_on()
            ax.grid(b=True, which='major', color='k', linestyle='-')
            ax.grid(b=True, which='minor', color='k', linestyle='-', alpha=0.2)
            ax.set_xlabel('time (sec)')
            ax.legend()

        pos_ax.set_ylabel('angle (rad)')
        vel_ax.set_ylabel('rate (rad/sec)')
        eff_ax.set_ylabel('effort (Nm)')
        pos_ax.set_title('joint states')

        if file_name is None:
            fig.show()
        else:
            fig.set_size_inches(width, height)
            fig.savefig(file_name, dpi=dpi)
            plt.close()
