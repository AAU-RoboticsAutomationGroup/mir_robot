# this is a launc file to work with the little helper igintion fortress sim
# which wust be launched seperatly due to ignition fortress overriding the 
# current gazebo version 


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    mir_nav_dir = get_package_share_directory('mir_navigation')
    
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


    mapping = Node(package='slam_toolbox',
                   executable='async_slam_toolbox_node',
                   parameters=[f"{mir_nav_dir}/config/mapping_fortress.yaml", {'use_sim_time':True}],
                   output='screen')

    rviz2 = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(mir_nav_dir, 'rviz', 'fortress_mapping.rviz')]
        )

    telop = Node(
                package='teleop_twist_keyboard',
                executable='teleop_twist_keyboard',
                prefix='xterm -e')
    
    
    return LaunchDescription([mapping,
                              laser_scan_merger,
                              rviz2,
                              telop])
