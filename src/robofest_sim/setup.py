import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robofest_sim'

def generate_data_files():
    data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'worlds/generated'), glob('worlds/generated/*.sdf') + glob('worlds/generated/*.json')),
    ]

    for root, dirs, files in os.walk('models'):
        if files:
            rel_path = os.path.relpath(root, 'models')
            target_path = os.path.join('share', package_name, 'models', rel_path)
            file_paths = [os.path.join(root, f) for f in files]
            data_files.append((target_path, file_paths))

    return data_files

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=generate_data_files(),
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='GPS-Denied Autonomous Drone Robofest Simulation Environment',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ground_truth_publisher = robofest_sim.ground_truth_publisher:main',
            'mission_evaluator = robofest_sim.mission_evaluator:main',
            'scenario_generator = robofest_sim.scenario_generator:main',
            'sensor_health_monitor = robofest_sim.sensor_health_monitor:main',
            'sim_tf_publisher = robofest_sim.sim_tf_publisher:main',
            'px4_ext_odom_node = robofest_sim.px4_ext_odom_node:main',
        ],
    },
)
