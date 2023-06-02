# Copyright 2018 Open Source Robotics Foundation, Inc.
# Copyright 2019 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
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

    arg_list = []
    ego_topic=LaunchConfiguration("ego_topic")
    opp_topic=LaunchConfiguration("opp_topic")

    ego_topic_arg = DeclareLaunchArgument("ego_topic", default_value="", description="topic name for ego odometry")
    opp_topic_arg = DeclareLaunchArgument("opp_topic", default_value="", description="topic name for opp odometry")
    
    use_sim_time=DeclareLaunchArgument("use_sim_time", default_value="false")
    arg_list.append(use_sim_time)
    arg_list.append(ego_topic_arg)
    arg_list.append(opp_topic_arg)

    node_list= []
    node_list.append(Node(
        package='racecar_utils',
        executable='odom_to_tf_node',
        name='odom_to_tf',
        remappings=[("odom_ego", ego_topic),
                    ("odom_opp", opp_topic)],
        parameters=[{use_sim_time.name: LaunchConfiguration(use_sim_time.name)}],
    ))

    node_list.append(Node(
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

    node_list.append(Node(
            package    = 'rviz2',
            namespace  = 'lead',
            executable = 'rviz2',
            name       = 'rviz2',
            arguments  = ['-d' + str(rviz_path)] 
    )),

    return LaunchDescription(arg_list+node_list)
