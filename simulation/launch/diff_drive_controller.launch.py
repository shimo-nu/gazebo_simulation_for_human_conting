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

def launch_setup(context, *arg, **kwargs):
  entity_name = LaunchConfiguration('robot_name', default='robot').perform(context)
  print(entity_name)
  diff_drive_node = Node(
      package="controller_manager",
      executable="spawner",
      arguments=[
          "diff_drive_controller",
      ],
      name="diff_drive_controller_" + entity_name,
      output='screen',
      namespace=entity_name
  )
  return [diff_drive_node]
  

def generate_launch_description():
  
  entity_name_arg = DeclareLaunchArgument('robot_name', default_value='robot', description='Name of the robot')
  

  return LaunchDescription([
    
    entity_name_arg,
    OpaqueFunction(function = launch_setup)
  ])
    