from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup(**generate_distutils_setup(
    packages=['ridgeback_bringup', 'ridgeback_bringup.multicast'],
    package_dir={'': 'src'}
))
