# cvall.launch.py

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """
    omo_r1mini bringup, 카메라, YOLO, 균열 탐지, 키보드 제어(teleop) 노드를 실행하는 런치 파일.
    """

    # ===================================================================================
    # == 로봇 기본 구동을 위한 omo_r1mini_bringup 런치 파일 포함 (가장 먼저 실행)
    # ===================================================================================
    omo_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('omo_r1mini_bringup'),
                'launch',
                'omo_r1mini_bringup.launch.py'
            )
        )
    )

    # ===================================================================================
    # == 직접 작성한 3개의 노드 실행
    # ===================================================================================
    # 1. 카메라 노드 (robot_camera)
    camera_node = Node(
        package='robot_camera_pkg',
        executable='robot_camera_node',
        name='camera_publisher',
        output='screen'
    )

    # 2. YOLO 알림 노드 (yolo_alert_pkg)
    yolo_node = Node(
        package='yolo_alert_pkg',
        executable='yolo_alert_node',
        name='yolo_alerter',
        output='screen'
    )

    # 3. 균열 탐지 노드 (crack_detector_pkg)
    crack_node = Node(
        package='crack_detector',
        executable='crack_detector_node',
        name='crack_detector',
        output='screen'
    )

    # ===================================================================================
    # == 키보드 제어(teleop) 노드 실행
    # ===================================================================================
    # 'ros2 run omo_r1mini_teleop teleop_keyboard'와 동일한 기능
    teleop_node = Node(
        package='omo_r1mini_teleop',
        executable='teleop_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e'  # 키보드 입력을 위해 새 터미널에서 실행
    )

    # ===================================================================================
    # == 최종 실행할 목록 반환 (순서대로 실행)
    # ===================================================================================
    return LaunchDescription([
        omo_bringup_launch,
        camera_node,
        yolo_node,
        crack_node,
        teleop_node
    ])
