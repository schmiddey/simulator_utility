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

  # ld = LaunchDescription()

  #launch configuration

  update_rate = 200.0
  step_size = 0.005
  viz_pub_rate = 30.0
  # use_sim_time = False

  frame_id = 'odom'
  topic    = 'map_sim'


  map           = LaunchConfiguration('map')
  use_sim_time  = LaunchConfiguration('use_sim_time')
  start_rviz    = LaunchConfiguration('start_rviz')
  # localization       = LaunchConfiguration('localization')
  nav           = LaunchConfiguration('nav')

  map_launch_arg = DeclareLaunchArgument(
    'map',
    default_value='sparse',
    choices=['sparse', 'robocup', 'hospital', 'kitchen'],
    description='Full path to map file to load'
  )

  use_sim_time_launch_arg = DeclareLaunchArgument(
    'use_sim_time',
    default_value='false',
    description='Use simulation (Gazebo) clock if true'
  )

  start_rviz_launch_arg = DeclareLaunchArgument(
    'start_rviz',
    default_value='true',
    description='Whether to start RViz'
  )

  localization_launch_arg = DeclareLaunchArgument(
    'loc',
    default_value='dummy',
    choices=['dummy', 'slam', 'amcl'],
    description='Which localization method to use'
  )

  nav_launch_arg = DeclareLaunchArgument(
    'nav',
    default_value='false',
    description='Whether to start navigation'
  )



  pre_path_wrld = os.path.join(get_package_share_directory('sim_launch'),'resources/flatland/world_robocook_')
  pre_path_map  = os.path.join(get_package_share_directory('sim_launch'),'resources/maps/')

  fltlnd_node = Node(package='flatland_server',
                       namespace='',
                       executable='flatland_server',
                       name='flatland_server',
                       output='log',
                      #  condition=LaunchConfigurationEquals('map', cond),
                       parameters=[{
                         "world_path"    : (TextSubstitution(text=pre_path_wrld), map, TextSubstitution(text='.yaml')),
                         "update_rate"   : update_rate,
                         "step_size"     : step_size,
                         "show_viz"      : False,
                         "viz_pub_rate"  : viz_pub_rate,
                         "use_sim_time"  : use_sim_time
                       }],
                       remappings=[
                         ('/cmd_vel', '/cmd_vel'),
                         ('/odometry/filtered', 'odom')
                       ]
                       )



  map_server_node = LifecycleNode(package='nav2_map_server',
                                  namespace='',
                                  executable='map_server',
                                  name='map_server_sim',
                                  output='screen',
                                  # condition=LaunchConfigurationEquals('map', cond),
                                  parameters=[{
                                    "frame_id"      : frame_id,
                                    "topic_name"    : topic,
                                    "use_sim_time"  : use_sim_time,
                                    "yaml_filename" : (TextSubstitution(text=pre_path_map), map, TextSubstitution(text='.yaml'))
                                  }]
                                  )


  # rviz_path=os.path.join(get_package_share_directory('sim_launch'), 'resources/sim.rviz')
  rviz2 = Node(package='rviz2',
               executable='rviz2',
               name='sim_rviz',
               condition=IfCondition(start_rviz),
              #  arguments=['-d', str(rviz_path)]  
               )


  static_tf_bl = Node(package='tf2_ros',
                   executable='static_transform_publisher',
                   name='static_transform_base_footprint_base_link',
                  #  condition = IfCondition(dummy_mapping),
                   arguments = ["0", "0", "0", "0", "0", "0", "base_footprint", "base_link"]  # '0 0 0 0 0 0 map odom'
                  #  arguments=''
                   )


  #twistmux -> like on robot
  robocook_mux_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sim_launch'),
                'launch/twist_mux.launch.py'))
  )


  #joy2vel node
  # joy2vel_ds4_node = IncludeLaunchDescription(
  #   PythonLaunchDescriptionSource(
  #           os.path.join(
  #               get_package_share_directory('todo'),
  #               'launch/joy2vel_todo.launch.py'))
  # )


  #########  localization  #########

  # #stfp  ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom          
  static_tf_dummy_mapping = Node(package='tf2_ros',
                                 executable='static_transform_publisher',
                                 name='static_transform_map_odom',
                                 condition = LaunchConfigurationEquals('loc', 'dummy'),
                                 arguments = ["0", "0", "0", "0", "0", "0", "map", "odom"]  # '0 0 0 0 0 0 map odom'
                                 )


  map_server_dummy_node = LifecycleNode(package='nav2_map_server',
                                  namespace='',
                                  executable='map_server',
                                  name='map_server_dummy',
                                  output='screen',
                                  condition=LaunchConfigurationEquals('loc', 'dummy'),
                                  parameters=[{
                                    "frame_id"      : 'map',
                                    "topic_name"    : 'map',
                                    "use_sim_time"  : use_sim_time,
                                    "yaml_filename" : (TextSubstitution(text=pre_path_map), map, TextSubstitution(text='.yaml'))
                                  }]
                                  )

  slam_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sim_launch'),
                'launch/slam.launch.py')),
    condition=LaunchConfigurationEquals('loc', 'slam')
  )


  #amcl

  map_server_amcl_node = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server_amcl',
            output='screen',
            condition=LaunchConfigurationEquals('loc', 'amcl'),
            # parameters=[configured_params],                                  
            parameters=[{
                         "frame_id"      : "map",
                         "topic_name"    : "map",
                         "use_sim_time"  : use_sim_time,
                         "yaml_filename"  : (TextSubstitution(text=pre_path_map), map, TextSubstitution(text='.yaml'))
                       }],
            # remappings=remappings
            )

  #to override use_sim_time from amcl
  param_substitutions = {
    'use_sim_time': use_sim_time,
  }
  configured_params = RewrittenYaml(
    source_file=os.path.join(get_package_share_directory('sim_launch'), 'config', 'amcl.yaml'),
    root_key='',
    param_rewrites=param_substitutions,
    convert_types=True
    )

  nav2_amcl_node = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            condition=LaunchConfigurationEquals('loc', 'amcl'),
            parameters=[configured_params],
            # remappings=remappings
            )

  nav2_manager_amcl_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_amcl',
            output='screen',
            condition=LaunchConfigurationEquals('loc', 'amcl'),
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server_amcl', 'amcl']}],
            )









  #nav
  nav_pkg = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sim_launch'),
                'launch/navigation.launch.py')),
    condition=IfCondition(nav)
  )


  # #emit configure event
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

  # #emit configure event
  configure_map_server_dummy_event = EmitEvent(
    event=ChangeState(
      lifecycle_node_matcher=matches_action(map_server_dummy_node),
      transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
    )
  )
  
  #emit active event after configure event is done (at state inactive)
  activate_map_server_dummy_event = RegisterEventHandler(
    OnStateTransition(
      target_lifecycle_node=map_server_node, goal_state='inactive',
      entities=[
        EmitEvent(event=ChangeState(
          lifecycle_node_matcher=matches_action(map_server_dummy_node),
          transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
        ))
      ]
    )
  )


  return LaunchDescription([
    #args
    map_launch_arg,
    use_sim_time_launch_arg,
    start_rviz_launch_arg,
    localization_launch_arg,
    nav_launch_arg,
    #nodes
    fltlnd_node,
    map_server_node,
    rviz2,
    static_tf_bl,
    robocook_mux_node,

    static_tf_dummy_mapping,
    map_server_dummy_node,

    slam_node,

    nav_pkg,

    map_server_amcl_node,
    nav2_amcl_node,
    nav2_manager_amcl_node,

    #events
    configure_map_server_event,
    activate_map_server_event,
    configure_map_server_dummy_event,
    activate_map_server_dummy_event
  ])