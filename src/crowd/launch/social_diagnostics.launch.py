from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    crowd_share = Path(
        get_package_share_directory('semaforr_crowd')
    )

    return LaunchDescription([
        Node(
            package='semaforr_crowd',
            executable='crowd_model',
            name='semaforr_crowd',
            output='screen',
            parameters=[str(crowd_share / 'config' / 'crowd.yaml')],
        ),
        Node(
            package='why',
            executable='why',
            name='why',
            output='screen',
        ),
        Node(
            package='why_plan',
            executable='why_plan',
            name='why_plan',
            output='screen',
        ),
    ])
