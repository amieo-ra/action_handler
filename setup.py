from setuptools import setup
import os
from glob import glob

package_name = 'action_handler'

setup(
    name=package_name,
    version='3.0.4',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['sympy>=1.5.1'],
    zip_safe=True,
    maintainer='Amie Owen',
    maintainer_email='amieo@live.co.uk',
    description='ROS1 action topic to ROS2 action conversion node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'action_handler.py = action_handler.action_handler:main'
        ],
    },

)
