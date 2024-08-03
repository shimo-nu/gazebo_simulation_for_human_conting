from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, SetLaunchConfiguration, LogInfo, ExecuteProcess, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

import os
import xacro


def launch_setup(context, *args, **kwargs):
  entity_name = LaunchConfiguration('robot_name').perform(context)
  x_spawn = LaunchConfiguration('x_spawn').perform(context)
  y_spawn = LaunchConfiguration('y_spawn').perform(context)
  z_spawn = LaunchConfiguration('z_spawn').perform(context)
  roll_spawn = LaunchConfiguration('roll_spawn').perform(context)
  pitch_spawn = LaunchConfiguration('pitch_spawn').perform(context)
  yaw_spawn = LaunchConfiguration('yaw_spawn').perform(context)

  # robot_description_topic_name = "/" + entity_name + "_robot_description"
  robot_description_topic_name = "/" + entity_name + "/robot_description"
  # robot_state_publisher_name= entity_name + "_robot_state_publisher"

  spawn_robot = Node(
      package='gazebo_ros',
      executable='spawn_entity.py',
      name='robot_spawn_entity_' + entity_name,
      output='screen',
      emulate_tty=True,
      arguments=['-entity',entity_name,
                  '-x', str(x_spawn), '-y', str(y_spawn), '-z', str(z_spawn),
                  '-R', str(roll_spawn), '-P', str(pitch_spawn), '-Y', str(yaw_spawn),
                  '-topic', robot_description_topic_name,
                  '-robot_namespace', entity_name
                  # '-gazebo_namespace', entity_name
                  ],
      # remappings=[("/robot_state_publisher", robot_state_publisher_name)
      #             ],
      namespace=entity_name
  )
  
    
  diff_drive_node = Node(
      package="controller_manager",
      executable="spawner",
      arguments=[
          "diff_drive_controller",
      ],
      output='screen',
      namespace=entity_name
  )
    
  return [spawn_robot]#, diff_drive_node]
  
def generate_launch_description(): 

  robot_name_arg = DeclareLaunchArgument('robot_name', default_value='robot')
  x_spawn_arg = DeclareLaunchArgument('x_spawn', default_value='0.0')
  y_spawn_arg = DeclareLaunchArgument('y_spawn', default_value='0.0')
  z_spawn_arg = DeclareLaunchArgument('z_spawn', default_value='0.0')
  roll_spawn_arg = DeclareLaunchArgument('roll_spawn', default_value='0.0')
  pitch_spawn_arg = DeclareLaunchArgument('pitch_spawn', default_value='0.0')
  yaw_spawn_arg = DeclareLaunchArgument('yaw_spawn', default_value='0.0')

  return LaunchDescription([
      robot_name_arg,
      x_spawn_arg,
      y_spawn_arg, 
      z_spawn_arg,
      roll_spawn_arg,
      pitch_spawn_arg,
      yaw_spawn_arg,
      OpaqueFunction(function = launch_setup)
      ])