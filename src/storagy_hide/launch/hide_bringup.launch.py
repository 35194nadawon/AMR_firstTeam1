#!/usr/bin/env python3
"""숨는팀 노드 일괄 실행.

사용:
  ros2 launch storagy_hide hide_bringup.launch.py
  ros2 launch storagy_hide hide_bringup.launch.py use_sim:=false   # 실하드웨어

베이스 시뮬(Gazebo+Nav2+YOLO+LLM)은 storagy full_bringup.launch.py 가 띄움.
이 런치는 그 위에 숨는팀 노드(R1~R4)만 얹는다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true',
                              description='true=시뮬 구현체, false=실하드웨어(MCU) 구현체'),

        Node(package='storagy_hide', executable='state_machine',
             name='hide_state_machine', output='screen'),
        Node(package='storagy_hide', executable='aruco_dock',
             name='hide_aruco_dock', output='screen'),
        Node(package='storagy_hide', executable='human_perception',
             name='hide_human_perception', output='screen'),
        Node(package='storagy_hide', executable='dynamic_costmap',
             name='hide_dynamic_costmap', output='screen'),
    ])
