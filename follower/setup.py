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
    maintainer='Francisco Nortes Novikov, Vicente Burdeus Sánchez',
    maintainer_email='fnornov@etsinf.upv.es, vbursan@etsinf.upv.es',
    description='Package that tracks the position of the predefined turtle that comes with turtlesim when you execute it without parameters and gives an explorer turtle the linear and angular velocity needed to reach the position of the default turtle.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    # Entry points: ejecutables disponibles después de la instalación
    entry_points={
        'console_scripts': [
                # Sistema principal: lanza todos los nodos (pose_saver, explorer_velocity, 
                # turtle_info_service, catch_info_action) con MultiThreadedExecutor
                'follower = follower.follower:main',
                
                # Cliente de servicio: consulta turtle_info cada 1 segundo y muestra la información
                'follower_server_client = follower.follower_server_client:main',
                
                # Cliente de acción: envía goal a catch_info y muestra feedback continuo
                'follower_action_client = follower.follower_action_client:main',
        ],
    },
)
