from setuptools import find_packages
from setuptools import setup

setup(
    name='humanoid_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('humanoid_interfaces', 'humanoid_interfaces.*')),
)
