from setuptools import setup, find_packages

package_name = 'package_manager_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'PyQt5',
    ],
    zip_safe=True,
    maintainer='YeonU',
    maintainer_email='yeonu0070@kw.ac.kr',
    description='Modular ROS2 Package Manager with PyQt5 GUI',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'package_manager = package_manager_py.main:main',
        ],
    },
)
