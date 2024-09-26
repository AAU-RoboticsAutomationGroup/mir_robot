
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

def generate_launch_description():

    mir_gazebo_dir = get_package_share_directory('mir_gazebo')
    mir_nav_dir = get_package_share_directory('mir_navigation')

    def find_map_file(context):
        map_arg = context.launch_configurations['map']
        print(f"trying to find map file with with arg {map_arg}")
        print(f"trying to find map at: {os.path.join(mir_nav_dir,'maps', map_arg)}")
        if os.path.isfile(os.path.join(mir_nav_dir, 'maps', map_arg)):
            print(f"map found in mir nav dir with path: \n{os.path.join(mir_nav_dir,'maps', map_arg)}")
            return [SetLaunchConfiguration('map_file', os.path.join(mir_nav_dir, 'maps', map_arg))]

        elif os.path.isfile(map_arg):
            print(f"using full map path")
            return [SetLaunchConfiguration('map_file', map_arg)]
    
    # declare_map_file_file_argument = DeclareLaunchArgument(
    #     'map_file',
    #     default_value=os.path.join(mir_nav_dir, 'maps', 'workspace1_map.yaml'),
    #     description='Relative path to map in mir_navigation/maps or full path to map (yaml).',
    # )
    #

    declare_map_file_argument = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(mir_nav_dir, 'maps','fib14_inmotiongrasping1e.yaml'),
        description='Relative path to map in mir_navigation/maps or full path to map (yaml).',
    )

    declare_rviz_config_file_argument = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(mir_nav_dir, 'rviz', 'fortress_nav.rviz'),
        description='Full path to rviz configuration file',
    )

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation/Gazebo clock'
    )

    declare_log_level = DeclareLaunchArgument(
        'log_level', default_value='info', description='the level of logging ie info, debug'
    )
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            get_package_share_directory("mir_navigation"), 'config', 'mapping_fortress.yaml'
        ),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node',
    )

    declare_nav_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(mir_nav_dir, 'config', 'nav_fortress_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    launch_amcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mir_nav_dir, 'launch', 'include', 'amcl.py')),
        launch_arguments={'map': LaunchConfiguration('map_file')}.items(),
    )

    launch_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mir_nav_dir, 'launch', 'include', 'navigation.py')),
        launch_arguments={'map_subscribe_transient_local': 'true'}.items(),
    )



    laser_scan_merger = Node(
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
                        'best_effort': False,
                        'use_sim_time':True,
                    }
                ],
                output='screen',
            )
    launch_rviz = Node(
        package='rviz2',
        executable='rviz2',
        output={'both': 'log'},
        arguments=['-d', os.path.join(mir_nav_dir, 'rviz', 'nav_fortress.rviz')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    ld = LaunchDescription()

    ld.add_action(declare_map_file_argument)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_log_level)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_nav_params_file_cmd)
    ld.add_action(declare_rviz_config_file_argument)
    # ld.add_action(declare_map_file_file_argument)
    ld.add_action(OpaqueFunction(function=find_map_file))
    ld.add_action(launch_amcl)
    ld.add_action(launch_navigation)
    ld.add_action(laser_scan_merger)
    ld.add_action(launch_rviz)

    return ld
