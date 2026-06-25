#!/usr/bin/env python3
"""숨는팀(토이 가이드, 컨셉 3) 노드 일괄 실행.

사용:
  ros2 launch storagy_hide hide_bringup.launch.py
  ros2 launch storagy_hide hide_bringup.launch.py use_sim:=false   # 실하드웨어(MCU)

베이스 시뮬(Gazebo+Nav2+YOLO+LLM+Web)은 storagy full_bringup.launch.py 가 띄운다.
이 런치는 FSM·ArUco 도킹만 얹는다. P3 사람감지는 기본 off:

  ros2 launch storagy_hide hide_bringup.launch.py enable_human_perception:=true

개발용 더미 신호(안내팀 없이 FREEZE→WAKE→GUIDE→RETURN→DOCK 전이를 테스트):
  ros2 topic pub /hide/takeover_start std_msgs/msg/Bool "{data: true}" --once  # 기상
  ros2 topic pub /hide/mission_done   std_msgs/msg/Bool "{data: true}" --once  # 복귀
  ros2 topic pub /hide/dock_done      std_msgs/msg/Bool "{data: true}" --once  # 은폐 완료
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('storagy_hide'), 'params', 'hide.yaml')

    use_sim = LaunchConfiguration('use_sim')
    enable_human_perception = LaunchConfiguration('enable_human_perception')

    common = {'output': 'screen', 'parameters': [params_file]}

    human_p3 = GroupAction(
        condition=IfCondition(enable_human_perception),
        actions=[
            Node(package='storagy_hide', executable='human_perception',
                 name='hide_human_perception', **common),
            Node(package='storagy_hide', executable='dynamic_costmap',
                 name='hide_dynamic_costmap', **common),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim', default_value='true',
            description='true=시뮬 구현체, false=실하드웨어(MCU) 구현체'),
        DeclareLaunchArgument(
            'enable_human_perception', default_value='false',
            description='true=P3 hide_human_perception·dynamic_costmap (기본 off)'),

        Node(package='storagy_hide', executable='state_machine',
             name='hide_state_machine', **common),
        Node(package='storagy_hide', executable='aruco_dock',
             name='hide_aruco_dock', **common),
        human_p3,
    ])
