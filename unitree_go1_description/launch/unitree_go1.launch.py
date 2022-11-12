#!/usr/bin/env -S python3

# MIT License
#
# Copyright (c) 2022 Naoki Takahashi
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros


def generate_launch_description():
    return launch.LaunchDescription(
        generate_declare_launch_arguments()
        + generate_launch_nodes()
    )


def generate_declare_launch_arguments():
    this_pkg_share_dir = get_package_share_directory('unitree_go1_description')

    return [
        launch.actions.DeclareLaunchArgument(
            'namespace',
            default_value=[''],
            description='Namespace (string)'
        ),
        launch.actions.DeclareLaunchArgument(
            'robot_model_file',
            default_value=['unitree_go1.urdf.xacro'],
            description='Robot model file (string)'
        ),
        launch.actions.DeclareLaunchArgument(
            'robot_model_path',
            default_value=[
                os.path.join(
                    this_pkg_share_dir,
                    'urdf'
                )
            ],
            description='Robot model file path (string)'
        ),
        launch.actions.DeclareLaunchArgument(
            'joint_state_publisher_config_file',
            default_value=[
                os.path.join(
                    this_pkg_share_dir,
                    'config',
                    'joint_state.yaml'
                )
            ],
            description='Joint state publisher configulation file of full path (string)'
        ),
        launch.actions.DeclareLaunchArgument(
            'use_sim_time',
            default_value=['false'],
            description='Use simulation time (boolean)'
        ),
        launch.actions.DeclareLaunchArgument(
            'ros2_control_config_file',
            default_value=[
                os.path.join(
                    this_pkg_share_dir,
                    'config',
                    'ros2_controllers.yaml'
                )
            ],
            description='ros2 controller config file path (string)'
        ),
        launch.actions.DeclareLaunchArgument(
            'use_real_hardware',
            default_value=['true'],
            description='Using real hardware control (boolean)',
        ),
        launch.actions.DeclareLaunchArgument(
            'ignition_gazebo',
            default_value=['false'],
            description='Using ignition gazebo (boolean)',
        ),
        launch.actions.DeclareLaunchArgument(
            'use_rviz',
            default_value=['false'],
            description='Using rviz2 (boolean)'
        ),
        launch.actions.DeclareLaunchArgument(
            'rviz_config_file',
            default_value=[
                os.path.join(
                    this_pkg_share_dir,
                    'rviz',
                    'unitree_go1.rviz'
                )
            ],
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('use_rviz')
            ),
            description='RViz2 config file of full path (string)'
        )
    ]


def generate_launch_nodes():
    # TODO Output parameter from declare launch arguemnt
    output = 'screen'

    urdf_file = launch.substitutions.PathJoinSubstitution([
        launch.substitutions.LaunchConfiguration('robot_model_path'),
        launch.substitutions.LaunchConfiguration('robot_model_file')
    ])

    use_sim_time = {
        'use_sim_time': launch.substitutions.LaunchConfiguration(
            'use_sim_time'
        )
    }
    robot_description = {
        'robot_description': launch.substitutions.Command([
            'xacro ',
            ' use_real_hardware:=',
            launch.substitutions.LaunchConfiguration('use_real_hardware'),
            ' ignition_gazebo:=',
            launch.substitutions.LaunchConfiguration('ignition_gazebo'),
            ' ros2_control_config_file:=',
            launch.substitutions.LaunchConfiguration('ros2_control_config_file'),
            ' ',
            urdf_file
        ])
    }

    exit_event = launch.actions.EmitEvent(
        event=launch.events.Shutdown()
    )

    return [
        launch.actions.GroupAction(actions=[
            launch_ros.actions.PushRosNamespace(
                namespace=launch.substitutions.LaunchConfiguration('namespace')
            ),
            launch_ros.actions.Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output=output,
                parameters=[
                    use_sim_time,
                    robot_description
                ],
                on_exit=exit_event
            ),
            launch_ros.actions.Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_merger',
                output=output,
                parameters=[
                    use_sim_time,
                    launch.substitutions.LaunchConfiguration(
                        'joint_state_publisher_config_file'
                    )
                ],
                on_exit=exit_event
            ),
            launch_ros.actions.Node(
                package='controller_manager',
                executable='ros2_control_node',
                output=output,
                parameters=[
                    use_sim_time,
                    robot_description,
                    launch.substitutions.LaunchConfiguration(
                        'ros2_control_config_file'
                    )
                ],
                remappings=[
                    ('joint_states', 'joint_state_broadcaster/joint_states')
                ],
                condition=launch.conditions.UnlessCondition(
                    launch.substitutions.LaunchConfiguration('ignition_gazebo')
                )
            ),
            launch_ros.actions.Node(
                package='controller_manager',
                executable='spawner',
                output=output,
                arguments=[
                    'joint_state_broadcaster',
                    '--controller-manager',
                    'controller_manager'
                ]
            ),
            launch_ros.actions.Node(
                package='controller_manager',
                executable='spawner',
                output=output,
                arguments=[
                    'joint_trajectory_controller',
                    '--controller-manager',
                    'controller_manager'
                ]
            ),
            launch_ros.actions.Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output=output,
                parameters=[
                    use_sim_time,
                ],
                arguments=[
                    '-d',
                    launch.substitutions.LaunchConfiguration(
                        'rviz_config_file'
                    )
                ],
                condition=launch.conditions.IfCondition(
                    launch.substitutions.LaunchConfiguration(
                        'use_rviz'
                    )
                )
            )
        ])
    ]
