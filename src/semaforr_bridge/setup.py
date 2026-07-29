from setuptools import find_packages, setup

package_name = 'semaforr_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eric Guan',
    maintainer_email='ericguan04@gmail.com',
    description='Stable ROS adapters for Social-SemaFORR.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_to_pose_bridge = semaforr_bridge.odom_to_pose_bridge:main',
            (
                'tracked_people_to_social_observation = '
                'semaforr_bridge.tracked_people_to_social_observation:main'
            ),
        ],
    },
)
