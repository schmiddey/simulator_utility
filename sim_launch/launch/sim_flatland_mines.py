import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle import matches_node_name
from launch_ros.event_handlers import OnStateTransition
from launch.events import matches_action

import lifecycle_msgs.msg



def generate_launch_description():
  map_server_node = LifecycleNode(package='nav2_map_server',
                                  namespace='',
                                  executable='map_server',
                                  name='map_server',
                                  output='screen',
                                  parameters=[{
                                    "frame_id"      : "map",
                                    "topic_name"    : "map",
                                    "use_sim_time"  : False,
                                    "yaml_filename"  : os.path.join(get_package_share_directory('sim_launch'), 'resources/mines.yaml')
                                  }]
                                  )


  flatland_server_node = Node(package='flatland_server',
                              namespace='',
                              executable='flatland_server',
                              name='flatland_server',
                              output='screen',
                              parameters=[{
                                "world_path"    : os.path.join(get_package_share_directory('sim_launch'), 'resources/flatland_world_mines.yaml'),
                                "update_rate"   : 200.0,
                                "step_size"     : 0.005,
                                "show_viz"      : False,
                                "viz_pub_rate"  : 30.0,
                              }]
                              )


  #stfp  ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom          
  static_tf = Node(package='tf2_ros',
                   executable='static_transform_publisher',
                   name='static_transform_map_odom',
                   arguments = ["0", "0", "0", "0", "0", "0", "map", "odom"]  # '0 0 0 0 0 0 map odom'
                  #  arguments=''
                   )

  #emit configure event
  configure_map_server_event = EmitEvent(
    event=ChangeState(
      lifecycle_node_matcher=matches_action(map_server_node),
      transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
    )
  )
  
  #emit active event after configure event is done (at state inactive)
  activate_map_server_event = RegisterEventHandler(
    OnStateTransition(
      target_lifecycle_node=map_server_node, goal_state='inactive',
      entities=[
        EmitEvent(event=ChangeState(
          lifecycle_node_matcher=matches_action(map_server_node),
          transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
        ))
      ]
    )
  )


  return LaunchDescription([
    static_tf,
    map_server_node,
    configure_map_server_event,
    activate_map_server_event,
    flatland_server_node
  ])