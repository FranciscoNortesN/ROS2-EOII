from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Francisco Nortes Novikov',
    maintainer_email='fnornov@etsinf.upv.es',
    description='Package that tracks the position of the predefined turtle that comes with turtlesim when you execute it without parameters and gives an explorer turtle the linear and angular velocity needed to reach the position of the default turtle.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
                'follower = follower.follower:main',
                'follower_server_client = follower.follower_server_client:main',
                'follower_action_client = follower.follower_action_client:main',
        ],
    },
)
