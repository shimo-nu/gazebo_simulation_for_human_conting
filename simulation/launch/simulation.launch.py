from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, SetLaunchConfiguration, LogInfo, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

import os

def generate_launch_description():
    # Declare the launch arguments
    launch_arguments = []
    node_list = []
    
    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    
    add_launch_arg('show_rviz', 'false')
    add_launch_arg('use_sim_time', 'true')
    add_launch_arg('gui', 'true')
    add_launch_arg('headless', 'false')
    add_launch_arg('debug', 'false')
    add_launch_arg('verbose', 'false')
    add_launch_arg('world_name', PathJoinSubstitution([
        FindPackageShare('journal1-sim'), 'world', 'people_count.world']))
    add_launch_arg('gpu', 'true')
        
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Load urdf 
    with open(os.path.join(get_package_share_directory('journal1-sim'), 'urdf', 'test.urdf'), 'r') as urdf_file:
        robot_description_content = urdf_file.read()


    xml = robot_description_content.replace('"', '\\"')
    spawn_args = '{name: \"my_robot\", xml: \"' + xml + '\" }'


    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world_name'), 'output': 'screen'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
        
    )
    
    node_list.extend([gzserver_cmd, gzclient_cmd])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description_content}]
    )
    
    # node_list.append(robot_state_publisher_node)

    
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        name='joint_state_controller',
        arguments=['joint_state_controller'],
    )
    
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description_content}],
        output='screen'
    )
    
    # gazebo_robot_spawner = Node(
    #   package='gazebo_ros',
    #   executable='spawn_entity.py',
    #   parameters=
    # )
    
    gazebo_robot_spawner = ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn_entity',
                 'gazebo_msgs/SpawnEntity', spawn_args]),
    
    # node_list.append(joint_state_spawner)
    node_list.append(gazebo_robot_spawner[0])
    # node_list.append(controller_manager)
    
    # Group for RViz and related configurations
    rviz_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('show_rviz')),
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                # output='screen',
                arguments=['-d', PathJoinSubstitution([
                    FindPackageShare('journal1-sim'), 'rviz', 'simulation_ros2.rviz'
                ])]
            )
        ]
    )
    
    node_list.append(rviz_group)
    return LaunchDescription(
      launch_arguments 
      + node_list)
