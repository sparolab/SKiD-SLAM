"""SKiD-SLAM, three robots (ROS 2 port of ros1/launch/run.launch).

  ros2 launch skid_slam run.launch.py config:=majang rviz:=true
  ros2 launch skid_slam run.launch.py fusion:=false   # without skid_slam_mapFusion
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

PROJECT = 'skid_slam'


def generate_launch_description():
    share = get_package_share_directory(PROJECT)

    config = LaunchConfiguration('config')
    params = LaunchConfiguration('params')
    robots = [LaunchConfiguration('robot0'), LaunchConfiguration('robot1'), LaunchConfiguration('robot2')]

    ld = LaunchDescription([
        # Dataset config: one of config/params/*.yaml (majang, planetary_analog)
        DeclareLaunchArgument('config', default_value='majang'),
        DeclareLaunchArgument('params', default_value=PathJoinSubstitution(
            [share, 'config', 'params', [config, '.yaml']])),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('fusion', default_value='true',
                              description='start skid_slam_mapFusion for every robot'),

        # Results (written on shutdown to <save_dir>/<robot>/)
        DeclareLaunchArgument('save_map', default_value='false'),
        DeclareLaunchArgument('save_poses', default_value='false'),
        DeclareLaunchArgument('save_scans', default_value='false'),
        DeclareLaunchArgument('save_whole_map', default_value='false'),
        DeclareLaunchArgument('save_dir', default_value='/tmp/skid_slam'),

        # Robot names must end with a digit; robot0 anchors the global map
        DeclareLaunchArgument('robot0', default_value='jackal0'),
        DeclareLaunchArgument('robot1', default_value='jackal1'),
        DeclareLaunchArgument('robot2', default_value='jackal2'),
    ])

    for i, robot in enumerate(robots):
        peers = [r for j, r in enumerate(robots) if j != i]
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'include', 'robot.launch.py')),
            launch_arguments={
                'robot': robot,
                'peers': [peers[0], ',', peers[1]],
                'params': params,
                'robot_initial': robots[0],
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'save_map': LaunchConfiguration('save_map'),
                'save_poses': LaunchConfiguration('save_poses'),
                'save_scans': LaunchConfiguration('save_scans'),
                'save_whole_map': LaunchConfiguration('save_whole_map'),
                'save_dir': LaunchConfiguration('save_dir'),
                'fusion': LaunchConfiguration('fusion'),
            }.items(),
        ))

    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='skid_slam_rviz',
        arguments=['-d', os.path.join(share, 'config', 'rviz', 'skid_slam.rviz')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    ))
    return ld
