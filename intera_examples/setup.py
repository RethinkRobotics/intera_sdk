#! /usr/bin/env python
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = ['intera_examples', 'intera_external_devices']
d['package_dir'] = {'': 'src'}

setup(**d)
