# Copyright 2024 Trossen Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from typing import List

from interbotix_xs_modules.xs_common import (
    get_interbotix_xsarm_models,
)
from interbotix_xs_modules.xs_launch import (
    declare_interbotix_xsarm_robot_description_launch_arguments,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import (
  IfCondition,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    robot_model_master_left_launch_arg = LaunchConfiguration('robot_model_master_left')
    robot_name_master_left_launch_arg = LaunchConfiguration('robot_name_master_left')
    robot_model_puppet_left_launch_arg = LaunchConfiguration('robot_model_puppet_left')
    robot_name_puppet_left_launch_arg = LaunchConfiguration('robot_name_puppet_left')

    robot_model_master_right_launch_arg = LaunchConfiguration('robot_model_master_right')
    robot_name_master_right_launch_arg = LaunchConfiguration('robot_name_master_right')
    robot_model_puppet_right_launch_arg = LaunchConfiguration('robot_model_puppet_right')
    robot_name_puppet_right_launch_arg = LaunchConfiguration('robot_name_puppet_right')

    use_puppet_rviz_launch_arg = LaunchConfiguration('use_puppet_rviz')
    launch_driver_launch_arg = LaunchConfiguration('launch_driver')
    use_sim_launch_arg = LaunchConfiguration('use_sim')
    xs_driver_logging_level_launch_arg = LaunchConfiguration('xs_driver_logging_level')

    xsarm_control_launch_master_left = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_control'),
                'launch',
                'xsarm_control.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model_master_left_launch_arg,
            'robot_name': robot_name_master_left_launch_arg,
            'base_link_frame': 'base_link',
            'use_rviz': 'false',
            'mode_configs': LaunchConfiguration('mode_configs_master_left'),
            'use_sim': use_sim_launch_arg,
            'robot_description': LaunchConfiguration('robot_description_master_left'),
            'xs_driver_logging_level': xs_driver_logging_level_launch_arg,
        }.items(),
        condition=IfCondition(launch_driver_launch_arg),
    )

    xsarm_control_launch_puppet_left = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_control'),
                'launch',
                'xsarm_control.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model_puppet_left_launch_arg,
            'robot_name': robot_name_puppet_left_launch_arg,
            'base_link_frame': 'base_link',
            'use_rviz': 'false',
            'mode_configs': LaunchConfiguration('mode_configs_puppet_left'),
            'use_sim': use_sim_launch_arg,
            'robot_description': LaunchConfiguration('robot_description_puppet_left'),
            'xs_driver_logging_level': xs_driver_logging_level_launch_arg,
        }.items(),
        condition=IfCondition(launch_driver_launch_arg),
    )

    xsarm_control_launch_master_right = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_control'),
                'launch',
                'xsarm_control.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model_master_right_launch_arg,
            'robot_name': robot_name_master_right_launch_arg,
            'base_link_frame': 'base_link',
            'use_rviz': 'false',
            'mode_configs': LaunchConfiguration('mode_configs_master_right'),
            'use_sim': use_sim_launch_arg,
            'robot_description': LaunchConfiguration('robot_description_master_right'),
            'xs_driver_logging_level': xs_driver_logging_level_launch_arg,
        }.items(),
        condition=IfCondition(launch_driver_launch_arg),
    )

    xsarm_control_launch_puppet_right = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_control'),
                'launch',
                'xsarm_control.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model_puppet_right_launch_arg,
            'robot_name': robot_name_puppet_right_launch_arg,
            'base_link_frame': 'base_link',
            'use_rviz': 'false',
            'mode_configs': LaunchConfiguration('mode_configs_puppet_right'),
            'use_sim': use_sim_launch_arg,
            'robot_description': LaunchConfiguration('robot_description_puppet_right'),
            'xs_driver_logging_level': xs_driver_logging_level_launch_arg,
        }.items(),
        condition=IfCondition(launch_driver_launch_arg),
    )


    xsarm_puppet_node_left = Node(
        name='xsarm_puppet_left',
        package='interbotix_aloha',
        executable='xsarm_puppet_left',
        output='screen',
        parameters=[
            {
                'robot_name_master_left': LaunchConfiguration('robot_name_master_left'),
                'robot_name_puppet_left': LaunchConfiguration('robot_name_puppet_left'),
            }
        ]
    )

    xsarm_puppet_node_right = Node(
        name='xsarm_puppet_right',
        package='interbotix_aloha',
        executable='xsarm_puppet_right',
        output='screen',
        parameters=[
            {
                'robot_name_master_left': LaunchConfiguration('robot_name_master_right'),
                'robot_name_puppet_left': LaunchConfiguration('robot_name_puppet_right'),
            }
        ]
    )

    tf_broadcaster_master_left = Node(
        name='tf_broadcaster_master_left',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '-0.50',
            '--y', '0.50',
            '--z', '0.0',
            '--frame-id', '/world',
            '--child-frame-id', (robot_name_master_left_launch_arg, '/base_link'),
        ]
    )

    tf_broadcaster_puppet_left = Node(
        name='tf_broadcaster_puppet_left',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.50',
            '--y', '0.50',
            '--z', '0.0',
            '--frame-id', '/world',
            '--child-frame-id', (robot_name_puppet_left_launch_arg, '/base_link'),
        ]
    )

    tf_broadcaster_master_right = Node(
        name='tf_broadcaster_master_right',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '-0.50',
            '--y', '-0.50',
            '--z', '0.0',
            '--frame-id', '/world',
            '--child-frame-id', (robot_name_master_right_launch_arg, '/base_link'),
        ]
    )

    tf_broadcaster_puppet_right = Node(
        name='tf_broadcaster_puppet_right',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.50',
            '--y', '-0.50',
            '--z', '0.0',
            '--frame-id', '/world',
            '--child-frame-id', (robot_name_puppet_right_launch_arg, '/base_link'),
        ]
    )

    rviz2_node = Node(
        condition=IfCondition(use_puppet_rviz_launch_arg),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', LaunchConfiguration('rvizconfig'),
        ],
        output={'both': 'log'},
    )

    return [
        xsarm_control_launch_master_left,
        xsarm_control_launch_puppet_left,
        xsarm_control_launch_master_right,
        xsarm_control_launch_puppet_right,
        xsarm_puppet_node_left,
        xsarm_puppet_node_right,
        tf_broadcaster_master_left,
        tf_broadcaster_puppet_left,
        tf_broadcaster_master_right,
        tf_broadcaster_puppet_right,
        rviz2_node,
    ]


def generate_launch_description():
    declared_arguments: List[DeclareLaunchArgument] = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model_master_left',
            default_value='wx250s', 
            choices=get_interbotix_xsarm_models(),
            description='model type of the master_left Interbotix Arm such as `wx250s`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name_master_left',
            default_value='master_left',
            description=(
                'name of the master_left robot.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model_puppet_left',
            default_value='vx300s', 
            choices=get_interbotix_xsarm_models(),
            description='model type of the puppet_left Interbotix Arm such as `vx300s`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name_puppet_left',
            default_value='puppet_left',
            description=(
                'name of the puppet_left robot.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'mode_configs_master_left',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_aloha'),
                'config',
                'master_modes_left.yaml',
            ]),
            description="the file path to the 'mode config' YAML file for the master_left arm.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'mode_configs_puppet_left',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_aloha'),
                'config',
                'puppet_modes_left.yaml',
            ]),
            description="the file path to the 'mode config' YAML file for the puppet_left arm.",
        )
    )



    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model_master_right',
            default_value='wx250s', 
            choices=get_interbotix_xsarm_models(),
            description='model type of the master_right Interbotix Arm such as `wx250s`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name_master_right',
            default_value='master_right',
            description=(
                'name of the master_right robot.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model_puppet_right',
            default_value='vx300s', 
            choices=get_interbotix_xsarm_models(),
            description='model type of the puppet_right Interbotix Arm such as `vx300s`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name_puppet_right',
            default_value='puppet_right',
            description=(
                'name of the puppet_right robot.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'mode_configs_master_right',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_aloha'),
                'config',
                'master_modes_right.yaml',
            ]),
            description="the file path to the 'mode config' YAML file for the master_right arm.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'mode_configs_puppet_right',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_aloha'),
                'config',
                'puppet_modes_right.yaml',
            ]),
            description="the file path to the 'mode config' YAML file for the puppet_right arm.",
        )
    )



    declared_arguments.append(
        DeclareLaunchArgument(
            'use_puppet_rviz',
            default_value='true',
            choices=('true', 'false'),
            description='launches RViz if set to `true`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_aloha'),
                'rviz',
                'stanford_aloha.rviz',
            ]),
            description='file path to the config file RViz should load.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'launch_driver',
            default_value='true',
            choices=('true', 'false'),
            description=(
                '`true` if xsarm_control should be launched - set to `false` if you would like to '
                'run your own version of this file separately.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'xs_driver_logging_level',
            default_value='INFO',
            choices=('DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL'),
            description='set the logging level of the X-Series Driver.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            choices=('true', 'false'),
            description=(
                "if `true`, the DYNAMIXEL simulator node is run; use RViz to visualize the robot's"
                ' motion; if `false`, the real DYNAMIXEL driver node is run.'
            ),
        )
    )
    declared_arguments.extend(
        declare_interbotix_xsarm_robot_description_launch_arguments(
            robot_description_launch_config_name='robot_description_master_left',
            robot_model_launch_config_name='robot_model_master_left',
            robot_name_launch_config_name='robot_name_master_left',
            use_world_frame='false',
        )
    )
    declared_arguments.extend(
        declare_interbotix_xsarm_robot_description_launch_arguments(
            robot_description_launch_config_name='robot_description_puppet_left',
            robot_model_launch_config_name='robot_model_puppet_left',
            robot_name_launch_config_name='robot_name_puppet_left',
            use_world_frame='false',
        )
    )

    declared_arguments.extend(
        declare_interbotix_xsarm_robot_description_launch_arguments(
            robot_description_launch_config_name='robot_description_master_right',
            robot_model_launch_config_name='robot_model_master_right',
            robot_name_launch_config_name='robot_name_master_right',
            use_world_frame='false',
        )
    )
    declared_arguments.extend(
        declare_interbotix_xsarm_robot_description_launch_arguments(
            robot_description_launch_config_name='robot_description_puppet_right',
            robot_model_launch_config_name='robot_model_puppet_right',
            robot_name_launch_config_name='robot_name_puppet_right',
            use_world_frame='false',
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
