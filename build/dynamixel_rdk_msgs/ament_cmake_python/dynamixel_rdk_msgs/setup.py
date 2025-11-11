from setuptools import find_packages
from setuptools import setup

setup(
    name='dynamixel_rdk_msgs',
    version='0.0.1',
    packages=find_packages(
        include=('dynamixel_rdk_msgs', 'dynamixel_rdk_msgs.*')),
)
