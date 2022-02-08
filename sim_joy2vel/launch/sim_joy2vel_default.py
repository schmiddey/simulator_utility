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
  sim_joy2vel_node = Node(package='sim_joy2vel',
                          namespace='',
                          executable='sim_joy2vel_ps4_node',
                          name='sim_joy2vel_ps4_node',
                          output='screen',
                          parameters=[{
                            "max_vel_lin_x" : 1.0,
                            "max_vel_lin_y" : 1.0, #not used
                            "max_vel_ang"   : 1.0,
                            "mode"          : "disdfsdff" #not used
                          }],
                          remappings=[
                            ('/cmd_vel', '/robot0/cmd_vel'),
                          ]
                          )

  joy_node = Node(package='joy',
                  namespace='',
                  executable='joy_node',
                  name='sim_joy_node'
                  )


  return LaunchDescription([
    sim_joy2vel_node,
    joy_node
  ])