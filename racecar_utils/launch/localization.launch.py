from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    share_dir = get_package_share_directory('racecar_utils')
    launch_dir = os.path.join(share_dir, 'launch')    
    urdf = os.path.join(launch_dir, 'av21.urdf')
    rviz_path = os.path.join(launch_dir, 'lidar_tf.rviz')

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    args = []
    namespace=DeclareLaunchArgument("ns", default_value="")
    args.append(namespace)
    use_sim_time=DeclareLaunchArgument("use_sim_time", default_value="false")
    args.append(use_sim_time)

    nodes= []

    nodes.append(Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        namespace=LaunchConfiguration(namespace.name),
        output='screen',
        remappings=[("/diagnostics",       "diagnostics"),
                    ("/odometry/filtered", "odometry/filtered"),
                    ("/accel/filtered",    "accel/filtered") ],
        parameters=[os.path.join(launch_dir, 'ekf.yaml'), {use_sim_time.name: LaunchConfiguration(use_sim_time.name)}]
    ))

    nodes.append(Node(
        package='racecar_utils',
        executable='odom_to_tf_node',
        name='odom_to_tf',
        namespace=LaunchConfiguration(namespace.name),
        remappings=[("odom_ego", 'odometry/filtered')],
        parameters=[{use_sim_time.name: LaunchConfiguration(use_sim_time.name)}],
    ))

    nodes.append(Node(
        package='racecar_utils',
        executable='convert_imu_node',
        name='convert_imu',
        namespace=LaunchConfiguration(namespace.name),
        remappings=[("imu_in", 'novatel_top/rawimu'),
                    ("imu_out", 'corrected_imu')],
        parameters=[{"gps_frame": "gps_top",use_sim_time.name: LaunchConfiguration(use_sim_time.name)}],
    ))

    nodes.append(Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name="robot_state_publisher",
    parameters=[{
        'publish_frequency': 1.0,
        'ignore_timestamp': False,
        'use_tf_static': True,
        'robot_description': robot_desc,
    }]
    ))

    nodes.append(Node(
            package    = 'rviz2',
            namespace  = 'lead',
            executable = 'rviz2',
            name       = 'rviz2',
            arguments  = ['-d' + str(rviz_path)] 
    )),


    return LaunchDescription(args+nodes)
