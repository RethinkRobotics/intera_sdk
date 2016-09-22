#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = ['intera_interface', 'intera_control', 'intera_dataflow',
                 'io_interface', 'joint_trajectory_action', 'gripper_action',
                 'head_action']
d['package_dir'] = {'': 'src'}

setup(**d)
