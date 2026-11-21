"""One robot: LiDAR-inertial odometry (LIO-SAM or FAST-LIO front end) + inter-robot map fusion.

Launch arguments:
  frontend       liosam (default: the LIO-SAM nodes) | fastlio (SOLiD's FAST-LIO SLAM: FAST-LIO + solid_node)
  fastlio_params FAST-LIO parameter file (frontend:=fastlio); with solid_config, starts FAST-LIO and solid_node
                 for this robot. Empty: read a FAST-LIO SLAM that already runs (fastlio_odom / fastlio_cloud)
  solid_config   SOLiD parameter file for solid_node (input extrinsic, keyframes, loop closing)
  lidar_topic, imu_topic        FAST-LIO input (default /<robot>/<pointCloudTopic>, /<robot>/<imuTopic>)
  fastlio_odom, fastlio_cloud   running FAST-LIO SLAM: loop-closed odometry, scans in the body frame
  robot          robot name, must end with a digit (e.g. jackal0)
  peers          other robots, comma separated (e.g. "jackal1,jackal2")
  params         parameter file (config/params/<config>.yaml)
  robot_initial  robot whose odom frame anchors the global map (mapfusion.robot_initial)
  use_sim_time, save_map, save_poses, save_scans, save_whole_map, save_dir
  fusion         start skid_slam_mapFusion (set false until that executable exists)
"""
import os

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

    frontend = lc('frontend')
    if frontend == 'liosam':
        front = [
            lio_node('imuPreintegration', True),
            lio_node('imageProjection', True),
            lio_node('featureExtraction', True),
            # Results are written on shutdown: give it time before SIGTERM/SIGKILL escalation.
            lio_node('mapOptmization', False, sigterm_timeout='60', sigkill_timeout='60'),
        ]
    elif frontend == 'fastlio':
        # SOLiD's FAST-LIO SLAM per robot (thirdparty/SOLiD): FAST-LIO (package fast_lio) + solid_node (package
        # solid_ros, loop closing), both in namespace <robot> with their own TF (<robot>/tf). FAST-LIO reads
        # lidar_topic / imu_topic (default <robot>/<pointCloudTopic>, <robot>/<imuTopic>). skid_slam_fastlioKeyframes
        # hands the keyframes (scans + solid_node's loop-closed pose) to map fusion.
        fastlio_params, solid_config = lc('fastlio_params'), lc('solid_config')
        front = []
        overrides = {k: v for k, v in (('fastlio.odom_topic', lc('fastlio_odom')),
                                        ('fastlio.cloud_topic', lc('fastlio_cloud'))) if v}
        if fastlio_params:
            import yaml  # noqa: PLC0415
            with open(params) as f:
                sk = (yaml.safe_load(f) or {}).get('/**', {}).get('ros__parameters', {}).get('skid_slam', {})
            lidar = lc('lidar_topic') or f'/{robot}/{sk.get("pointCloudTopic", "points_raw")}'
            imu = lc('imu_topic') or f'/{robot}/{sk.get("imuTopic", "imu_raw")}'
            if not solid_config:
                raise RuntimeError('frontend:=fastlio with fastlio_params also needs solid_config (SOLiD YAML)')
            sim = {'use_sim_time': LaunchConfiguration('use_sim_time')}
            tf = [('/tf', f'/{robot}/tf'), ('/tf_static', f'/{robot}/tf_static')]
            front += [
                Node(package='fast_lio', executable='fastlio_mapping', name='fastlio', namespace=robot,
                     parameters=[fastlio_params, sim, {'common.lid_topic': lidar, 'common.imu_topic': imu,
                                                       'publish.scan_publish_en': True,
                                                       'publish.scan_bodyframe_pub_en': True}],
                     remappings=[('/Odometry', f'/{robot}/fastlio/odometry'),
                                 ('/cloud_registered', f'/{robot}/fastlio/cloud_registered'),
                                 ('/cloud_registered_body', f'/{robot}/fastlio/cloud_registered_body'),
                                 ('/cloud_effected', f'/{robot}/fastlio/cloud_effected'),
                                 ('/Laser_map', f'/{robot}/fastlio/map'),
                                 ('/path', f'/{robot}/fastlio/path')] + tf,
                     output='screen'),
                Node(package='solid_ros', executable='solid_node', name='solid', namespace=robot,
                     parameters=[sim, {'config': solid_config,
                                       'save_dir': os.path.join(lc('save_dir'), robot, 'solid')}],
                     remappings=[('cloud', lidar), ('odom', f'/{robot}/fastlio/odometry')] + tf,
                     output='screen'),
            ]
            overrides = {'fastlio.odom_topic': f'/{robot}/solid/odometry',
                         'fastlio.cloud_topic': f'/{robot}/fastlio/cloud_registered_body', **overrides}
        front.append(Node(
            package=PROJECT, executable=f'{PROJECT}_fastlioKeyframes', name=f'{PROJECT}_fastlioKeyframes_{robot}',
            parameters=common + [overrides], output='screen',
            sigterm_timeout='60', sigkill_timeout='60'))
    else:
        raise RuntimeError(f"frontend must be liosam or fastlio, got '{frontend}'")

    return front + [
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
        DeclareLaunchArgument('frontend', default_value='liosam', description='liosam | fastlio'),
        DeclareLaunchArgument('fastlio_params', default_value='',
                              description='FAST-LIO parameter file; empty: use a running FAST-LIO'),
        DeclareLaunchArgument('solid_config', default_value='', description='SOLiD YAML for solid_node'),
        DeclareLaunchArgument('lidar_topic', default_value='', description='FAST-LIO LiDAR input'),
        DeclareLaunchArgument('imu_topic', default_value='', description='FAST-LIO IMU input'),
        DeclareLaunchArgument('fastlio_odom', default_value='',
                              description='loop-closed odometry of a running FAST-LIO SLAM (solid/odometry)'),
        DeclareLaunchArgument('fastlio_cloud', default_value='',
                              description='body-frame scans (cloud_registered_body) of a running FAST-LIO'),
        OpaqueFunction(function=_nodes),
    ])
