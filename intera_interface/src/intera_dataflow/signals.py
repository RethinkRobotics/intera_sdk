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

import inspect
from weakref import WeakKeyDictionary

try:
    from weakref import WeakSet
except ImportError:
    from weakrefset import WeakSet


class Signal(object):
    def __init__(self):
        self._functions = WeakSet()
        self._methods = WeakKeyDictionary()

    def __call__(self, *args, **kargs):
        for f in self._functions:
            f(*args, **kargs)

        for obj, functions in self._methods.items():
            for f in functions:
                f(obj, *args, **kargs)

    def connect(self, slot):
        if inspect.ismethod(slot):
            if not slot.__self__ in self._methods:
                self._methods[slot.__self__] = set()
            self._methods[slot.__self__].add(slot.__func__)
        else:
            self._functions.add(slot)

    def disconnect(self, slot):
        if inspect.ismethod(slot):
            if slot.__self__ in self._methods:
                self._methods[slot.__self__].remove(slot.__func__)
        else:
            if slot in self._functions:
                self._functions.remove(slot)
