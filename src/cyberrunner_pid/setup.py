from setuptools import find_packages, setup

package_name = 'cyberrunner_pid'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='trungbao',
    maintainer_email='truongtrungbao030102.us@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'pid_controller = cyberrunner_pid.pid_controller:main',
        'route_wfocus_follow = cyberrunner_pid.route_wfocus_follow:main',
        'click_target = cyberrunner_pid.click_target:main',
'maze_planner = cyberrunner_pid.maze_planner:main',
            "orange_track_mapper_node = cyberrunner_pid.orange_track_mapper_node:main",
            "path_follower_node = cyberrunner_pid.path_follower_node:main",
            "orange_live_target_node = cyberrunner_pid.orange_live_target_node:main",
            "solve = cyberrunner_pid.solve:main",

    ],
},

)
