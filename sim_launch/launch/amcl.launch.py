import os
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import LogInfo, DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, LaunchConfigurationEquals
import lifecycle_msgs.msg
from nav2_common.launch import RewrittenYaml


def generate_launch_description():


  map           = LaunchConfiguration('map')

  map_launch_arg = DeclareLaunchArgument(
    'map',
    default_value='/home/user/maps/map.yaml',
    # choices=['sparse', 'robocup', 'hospital', 'kitchen'],
    description='Full path to map file to load'
  )


  map_server_amcl_node = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server_amcl',
            output='screen',
            # condition=LaunchConfigurationEquals('loc', 'amcl'),
            # parameters=[configured_params],                                  
            parameters=[{
                         "frame_id"      : "map",
                         "topic_name"    : "map",
                         "use_sim_time"  : False,
                         "yaml_filename"  : map
                       }],
            # remappings=remappings
            )

  #to override use_sim_time from amcl
  param_substitutions = {
    'use_sim_time': "false",
  }

  configured_params = RewrittenYaml(
    source_file=os.path.join(get_package_share_directory('thi_rc_launch'), 'config', 'amcl.yaml'),
    root_key='',
    param_rewrites=param_substitutions,
    convert_types=True
    )

  nav2_amcl_node = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            # condition=LaunchConfigurationEquals('loc', 'amcl'),
            parameters=[configured_params],
            # remappings=remappings
            )

  nav2_manager_amcl_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_amcl',
            output='screen',
            # condition=LaunchConfigurationEquals('loc', 'amcl'),
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': ['map_server_amcl', 'amcl']}],
            )



  return LaunchDescription([
    map_launch_arg,

    map_server_amcl_node,
    nav2_amcl_node,
    nav2_manager_amcl_node
  ])

