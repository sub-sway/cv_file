# all.launch.py

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    사용자의 모든 노드와 자율주행 관련 런치 파일을 한 번에 실행하는 최상위 런치 파일.
    """

    # ===================================================================================
    # == 런치 파일 인자 선언 (map 파일 경로)
    # ===================================================================================
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value='/home/orin/porty2.yaml',
        description='Full path to map file to load'
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
    # == 기존 런치 파일 3개 포함
    # ===================================================================================
    # 1. omo_r1mini_bringup
    omo_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('omo_r1mini_bringup'),
                'launch',
                'omo_r1mini_bringup.launch.py'
            )
        )
    )

    # 2. omo_r1mini_navigation2
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('omo_r1mini_navigation2'),
                'launch',
                'navigation2.launch.py'
            )
        ),
        launch_arguments={'map': LaunchConfiguration('map_path')}.items()
    )

    # 3. navigation2_rviz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('omo_r1mini_navigation2'),
                'launch',
                'navigation2_rviz.launch.py'
            )
        )
    )

    # ===================================================================================
    # == 최종 실행할 목록 반환
    # ===================================================================================
    return LaunchDescription([
        map_path_arg,
        camera_node,
        yolo_node,
        crack_node,
        omo_bringup_launch,
        navigation_launch,
        rviz_launch
    ])
