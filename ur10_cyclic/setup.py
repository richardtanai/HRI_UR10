import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ur10_cyclic'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin_arduino_monitor.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='richard',
    maintainer_email='richardtanai@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mover = ur10_cyclic.cyclic_mover:main',
            'recorder = ur10_cyclic.recorder:main',
            'human_safety = ur10_cyclic.human_safety_node:main',
            'camera_tf_pub = ur10_cyclic.camera_tf_publisher:main',
            'arduino_bridge = ur10_cyclic.arduino_bridge:main',
            'live_plotter = ur10_cyclic.live_plotter:main',
            'aruco_sequence = ur10_cyclic.aruco_sequence:main',
            'sequence_controller = ur10_cyclic.sequence_controller:main',
        ],
    },
    scripts = [
        'scripts/rqt_arduino_monitor.py',
    ],
)
