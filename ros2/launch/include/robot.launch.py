"""One robot: LiDAR-inertial odometry (LIO-SAM based) + inter-robot map fusion.

Launch arguments:
  robot          robot name, must end with a digit (e.g. jackal0)
  peers          other robots, comma separated (e.g. "jackal1,jackal2")
  params         parameter file (config/params/<config>.yaml)
  robot_initial  robot whose odom frame anchors the global map (mapfusion.robot_initial)
  use_sim_time, save_map, save_poses, save_scans, save_whole_map, save_dir
  fusion         start skid_slam_mapFusion (set false until that executable exists)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PROJECT = 'skid_slam'


def _nodes(context):
    lc = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731

    robot = lc('robot')
    peers = [p.strip() for p in lc('peers').replace('[', '').replace(']', '').split(',') if p.strip()]
    params = lc('params')

    common = [
        params,
        {
            'robot_id': robot,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'skid_slam.saveMap': LaunchConfiguration('save_map'),
            'skid_slam.savePoses': LaunchConfiguration('save_poses'),
            'skid_slam.saveScans': LaunchConfiguration('save_scans'),
            'skid_slam.saveWholeMap': LaunchConfiguration('save_whole_map'),
            'skid_slam.saveDirectory': LaunchConfiguration('save_dir'),
        },
    ]

    def lio_node(name, respawn, **kwargs):
        return Node(
            package=PROJECT,
            executable=f'{PROJECT}_{name}',
            name=f'{PROJECT}_{name}_{robot}',
            parameters=common,
            output='screen',
            respawn=respawn,
            **kwargs,
        )

    return [
        lio_node('imuPreintegration', True),
        lio_node('imageProjection', True),
        lio_node('featureExtraction', True),
        # Results are written on shutdown: give it time before SIGTERM/SIGKILL escalation.
        lio_node('mapOptmization', False, sigterm_timeout='60', sigkill_timeout='60'),
        Node(
            package=PROJECT,
            executable=f'{PROJECT}_mapFusion',
            name=f'{PROJECT}_mapFusion_{robot}',
            parameters=[
                params,
                {
                    'robot_id': robot,
                    'peers': peers,
                    'pcm_matrix_dir': '/tmp',
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'mapfusion.robot_initial': LaunchConfiguration('robot_initial'),
                },
            ],
            output='screen',
            respawn=False,
            condition=IfCondition(LaunchConfiguration('fusion')),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot'),
        DeclareLaunchArgument('peers', default_value=''),
        DeclareLaunchArgument('params'),
        DeclareLaunchArgument('robot_initial', default_value='jackal0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('save_map', default_value='false'),
        DeclareLaunchArgument('save_poses', default_value='false'),
        DeclareLaunchArgument('save_scans', default_value='false'),
        DeclareLaunchArgument('save_whole_map', default_value='false'),
        DeclareLaunchArgument('save_dir', default_value='/tmp/skid_slam'),
        DeclareLaunchArgument('fusion', default_value='true'),
        OpaqueFunction(function=_nodes),
    ])
