from setuptools import find_packages, setup

package_name = 'social_context'

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
    description='Social observation and trajectory prediction producers.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'social_context_hunav = social_context.social_context_hunav:main',
            (
                'image_publisher = '
                'social_context.pose_estimation.tests.image_publisher:main'
            ),
            (
                'tf_human_matcher = '
                'social_context.pose_estimation.tests.tf_human_matcher:main'
            ),
            (
                'person_relative_localizer = '
                'social_context.pose_estimation.src.'
                'person_relative_localizer:main'
            ),
            # 'openpose_node = social_context.pose_estimation.src.openpose_node:main',
            # 'mediapipe_pose_node = social_context.pose_estimation.src.mediapipe_pose_node:main',
            (
                'pose_openpose = social_context.pose_estimation.src.'
                'camera_2d_pose_detection_node:main_openpose'
            ),
            (
                'pose_mediapipe = social_context.pose_estimation.src.'
                'camera_2d_pose_detection_node:main_mediapipe'
            ),
            (
                'global_human_localizer = '
                'social_context.pose_estimation.src.'
                'global_human_localizer:main'
            ),
            (
                'sort_tracker = social_context.pose_estimation.src.'
                'tracking.sort_node:main'
            ),
            (
                'tracking_evaluator = '
                'social_context.pose_estimation.src.tracking.'
                'tracking_evaluator:main'
            ),
        ],
    },
)
