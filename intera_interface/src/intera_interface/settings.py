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

JOINT_ANGLE_TOLERANCE = 0.008726646
HEAD_PAN_ANGLE_TOLERANCE = 0.1

## Versioning
SDK_VERSION = '5.2.0'
CHECK_VERSION = True
# Version Compatibility Maps - {current: compatible}
VERSIONS_SDK2ROBOT = {'5.1.0': ['5.1.0', '5.1.1', '5.1.2'],
                      '5.2.0': ['5.2.0', '5.2.1', '5.2.2']
                     }
VERSIONS_SDK2GRIPPER = {'5.2.0':
                          {
                           'warn': '2014/5/20 00:00:00',  # Version 1.0.0
                           'fail': '2013/10/15 00:00:00', # Version 0.6.2
                          }
                       }
