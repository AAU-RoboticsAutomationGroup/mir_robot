# Copyright (c) 2018-2022, Martin Günther (DFKI GmbH) and contributors
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
#
# Author: relffok

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    mir_description_dir = get_package_share_directory('mir_description')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription(
        [
            DeclareLaunchArgument('namespace', default_value='', description='Namespace to push all topics into.'),
            DeclareLaunchArgument('use_sim_time', default_value='false', description=''),
            DeclareLaunchArgument('mir_hostname', default_value='192.168.12.20', description=''),
            # DeclareLaunchArgument(
            #     'disable_map',
            #     default_value='false',
            #     description='Disable the map topic and map -> odom_comb TF transform from the MiR'),
            DeclareLaunchArgument(
                'robot_state_publisher_enabled',
                default_value='true',
                description='Set to true to publish tf using mir_description',
            ),
            #IncludeLaunchDescription(
            #    FrontendLaunchDescriptionSource(
            #        os.path.join(mir_description_dir, 'launch', 'robot_state_publisher.launch')
            #    ),
            #    launch_arguments={
            #        'tf_prefix': LaunchConfiguration('namespace'),
            #    }.items(),
            #    condition=IfCondition(LaunchConfiguration('robot_state_publisher_enabled')),
            #),
            Node(
                package='mir_driver',
                executable='mir_bridge',
                parameters=[
                    {'use_sim_time': LaunchConfiguration('use_sim_time'), 'tf_prefix': LaunchConfiguration('namespace')}
                ],
                namespace=LaunchConfiguration('namespace'),
                output='screen',
            ),
            Node(
                package='mir_driver',
                executable='fake_mir_joint_publisher',
                remappings=[('use_sim_time', LaunchConfiguration('use_sim_time'))],
                parameters=[{'tf_prefix': LaunchConfiguration('namespace')}],
                namespace=LaunchConfiguration('namespace'),
                output='screen',
            ),
            Node(
                package='twist_stamper',
                executable='twist_stamper',
                name='twist_stamper_cmd_vel_mir',
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                remappings=[
                    ('cmd_vel_in', 'cmd_vel'),
                    ('cmd_vel_out', 'cmd_vel_stamped'),
                ],
                namespace=LaunchConfiguration('namespace'),
            ),
            Node(
                package='ira_laser_tools',
                name='mir_laser_scan_merger',
                executable='laserscan_multi_merger',
                parameters=[
                    {
                        'laserscan_topics': "b_scan f_scan",
                        'destination_frame': "virtual_laser_link",
                        'scan_destination_topic': 'scan',
                        'cloud_destination_topic': 'scan_cloud',
                        'min_height': -0.25,
                        'max_merge_time_diff': 0.05,
                        # driver (msg converter) delay
                        'max_delay_scan_time': 2.5,
                        'max_completion_time': 0.1,
                        'alow_scan_delay': True,
                        'use_sim_time': use_sim_time,
                        'best_effort': False,
                    }
                ],
                namespace=LaunchConfiguration('namespace'),
                output='screen',
            ),
        ]
    )
