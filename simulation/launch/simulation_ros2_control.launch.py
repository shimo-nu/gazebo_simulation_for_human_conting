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

def robot_settings(robot_name, robot_namespace):
    node_list = []
    # Load urdf
    share_dir_path = os.path.join(get_package_share_directory('journal1-sim'))
    xacro_path = os.path.join(share_dir_path, 'urdf', 'smagv', 'smagv.urdf.xacro')
    urdf_path = os.path.join(share_dir_path, 'urdf', 'smagv', 'smagv.urdf')

    doc = xacro.process_file(xacro_path)
    robot_description = doc.toprettyxml(indent='  ')  
    with open(urdf_path, 'w') as f:
        f.write(robot_description)
        
    xml = robot_description.replace('"', '\\"')
    # spawn_args = '{name: \"my_robot\", xml: \"' + xml + '\",robot_namespace: \"robot_1\"}'
    spawn_args = '{name: \"' + robot_name + '\", xml: \"' + xml + '\", robot_namespace: \"' + robot_namespace + '\"}'


    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
        namespace=robot_namespace
    )
    
    # Joint state publisher
    joint_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        namespace=robot_namespace
    )

    
    node_list.extend([robot_state_publisher_node, joint_publisher])
    
    
    diff_drive_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
        ],
        output='screen',
        namespace=robot_namespace
    )
    
    
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )
    
    
    gazebo_robot_spawner = ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn_entity',
                'gazebo_msgs/SpawnEntity', spawn_args]),
    
    
    node_list.append(gazebo_robot_spawner[0])
    node_list.append(diff_drive_node)
    # node_
    
    
    return node_list


def generate_launch_description():
    # Declare the launch arguments
    launch_arguments = []
    node_list = []
    
    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))


   
    
    
    add_launch_arg('show_rviz', 'true')
    add_launch_arg('use_sim_time', 'true')
    add_launch_arg('gui', 'true')
    add_launch_arg('headless', 'false')
    add_launch_arg('debug', 'false')
    add_launch_arg('verbose', 'false')
    add_launch_arg('world_name', PathJoinSubstitution([
        FindPackageShare('journal1-sim'), 'world', 'many_people.world']))
    add_launch_arg('gpu', 'true')
        
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Load urdf 
    # with open(os.path.join(get_package_share_directory('journal1-sim'), 'urdf', 'smagv', 'smagv.urdf'), 'r') as urdf_file:
    # # with open(os.path.join(get_package_share_directory('journal1-sim'), 'urdf', 'test.urdf'), 'r') as urdf_file:
    #     robot_description_content = urdf_file.read()


    # xml = robot_description_content.replace('"', '\\"')
    # spawn_args = '{name: \"my_robot\", xml: \"' + xml + '\" }'


    # share_dir_path = os.path.join(get_package_share_directory('journal1-sim'))
    # xacro_path = os.path.join(share_dir_path, 'urdf', 'smagv', 'smagv.urdf.xacro')
    # urdf_path = os.path.join(share_dir_path, 'urdf', 'smagv', 'smagv.urdf')

    # doc = xacro.process_file(xacro_path)
    # robot_description = doc.toprettyxml(indent='  ')  
    # with open(urdf_path, 'w') as f:
    #     f.write(robot_description)
        
    # xml = robot_description.replace('"', '\\"')
    # # spawn_args = '{name: \"my_robot\", xml: \"' + xml + '\",robot_namespace: \"robot_1\"}'
    # spawn_args = '{name: \"my_robot\", xml: \"' + xml + '\"}'

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world_name'), 'output': 'screen'}.items()
    )

    
    
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        launch_arguments={'use_respawn': 'true'}.items()
    )
    
    
    
    node_list.extend([gzserver_cmd])

    node_list.extend(robot_settings(robot_name='robot_1', robot_namespace='robot1',))
    
    launch_gazebo_client = lambda _: [gzclient_cmd]
    
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
    
    static_transform_publisher = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'odom']
        )
    
    # node_list.append(static_transform_publisher)
    
    node_list.append(rviz_group)
    return LaunchDescription(
      launch_arguments 
      + node_list)
    #   + [OpaqueFunction(function=launch_gazebo_client)])
