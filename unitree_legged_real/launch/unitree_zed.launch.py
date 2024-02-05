
"""
Launches high level controls for the unitree go1, the zed camera, and the zed configured rviz environment.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    OpaqueFunction,
    IncludeLaunchDescription
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution,
    PathJoinSubstitution
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context, *args, **kwargs):

    # Launch configuration variables
    start_zed_node = LaunchConfiguration('start_zed_node')
    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')

    camera_name_val = camera_name.perform(context)
    camera_model_val = camera_model.perform(context)

    if (camera_name_val == ''):
        camera_name_val = 'zed'

    # Rviz2 Configurations to be loaded by ZED Node
    config_rviz2 = os.path.join(
        get_package_share_directory('zed_display_rviz2'),
        'rviz2',
        camera_model_val + '.rviz'
    )

    # Rviz2 node
    rviz2_node = Node(
        package='rviz2',
        namespace=camera_name_val,
        executable='rviz2',
        name=camera_model_val +'_rviz2',
        output='screen',
        arguments=[['-d'], [config_rviz2]],
    )

    # ZED Wrapper launch file
    zed_wrapper_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('zed_wrapper'),
            '/launch/zed_camera.launch.py'
        ]),
        launch_arguments={
            'camera_name': camera_name_val,
            'camera_model': camera_model_val
        }.items(),
        condition=IfCondition(start_zed_node)
    )

    # Start udp_high
    udp_high = Node(
            package='unitree_legged_real',
            executable='udp_high',
            output='screen'
        )

    # Start jsp_high
    jsp_high = Node(
        package='unitree_legged_real',
        executable='jsp_high',
        output='screen'
    )

    # Start waypoint navigation node
    unitree_waypoint = Node(
        package='unitree_legged_real',
        executable='unitree_waypoint',
        output='screen'
    )


    return [
        rviz2_node,
        zed_wrapper_launch,
        udp_high,
        jsp_high,
        unitree_waypoint
    ]

def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            name='use_rviz',
            default_value='false',
            choices=['true','false'],
            description='Open RVIZ for Go1 visualization'
        ),

        DeclareLaunchArgument(
            name='fixed_frame',
            default_value='base_footprint',
            description='Fixed frame for RVIZ'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('go1_description'),
                    'launch',
                    'load_go1.launch.py'
                ])
            ),
            launch_arguments=[
                ('use_jsp', 'none'),
                ('fixed_frame', LaunchConfiguration('fixed_frame')),
                ('use_nav2_links', 'true'),
                ('use_rviz', LaunchConfiguration('use_rviz')),
            ],
        ),

        DeclareLaunchArgument(
            'start_zed_node',
            default_value='True',
            description='Set to `False` to start only Rviz2 if a ZED node is already running.'),

        DeclareLaunchArgument(
            'camera_name',
            default_value=TextSubstitution(text='zed'),
            description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.'),

        DeclareLaunchArgument(
            'camera_model',
            default_value=TextSubstitution(text='zed2i'),
            description='[REQUIRED] The model of the camera. Using a wrong camera model can disable camera features.',
            choices=['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm']),

        OpaqueFunction(function=launch_setup)
    ])