#!/usr/bin/env -S python3

# BSD 3-Clause License
#
# Copyright (c) 2023 NaokiTakahashi.
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
#    * Neither the name of the {copyright_holder} nor the names of its
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
            'use_gz',
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

    gz_version_env_name = 'IGNITION_VERSION'
    ign_compatible = ''

    if os.getenv(gz_version_env_name) is None:
        gz_version_env_name = 'GZ_VERSION'
        if os.getenv(gz_version_env_name) is None:
            raise KeyError('Please export ' + gz_version_env_name)
    if os.getenv(gz_version_env_name) == 'garden':
        ign_compatible = 'ign_compatible:=false'
    else:
        ign_compatible = 'ign_compatible:=true'

    robot_description = {
        'robot_description': launch.substitutions.Command([
            'xacro ',
            ' use_real_hardware:=',
            launch.substitutions.LaunchConfiguration('use_real_hardware'),
            ' use_gz:=',
            launch.substitutions.LaunchConfiguration('use_gz'),
            ' ros2_control_config_file:=',
            launch.substitutions.LaunchConfiguration('ros2_control_config_file'),
            ' ', ign_compatible,
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
                    launch.substitutions.LaunchConfiguration('use_gz')
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
