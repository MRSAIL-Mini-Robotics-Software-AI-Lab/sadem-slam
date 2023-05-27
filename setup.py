"""
Install the sadem_slam library
"""
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(packages=["sadem_slam"], package_dir={"": "src"})

setup(**d)
