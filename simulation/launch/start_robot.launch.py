import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    # Accessing the argument variables.
    robot_name = LaunchConfiguration('robot_name').perform(context)
    robot_file = LaunchConfiguration('robot_file').perform(context)
    
    x_spawn = LaunchConfiguration('x_spawn').perform(context)
    y_spawn = LaunchConfiguration('y_spawn').perform(context)
    z_spawn = LaunchConfiguration('z_spawn').perform(context)
    roll_spawn = LaunchConfiguration('roll_spawn').perform(context)
    pitch_spawn = LaunchConfiguration('pitch_spawn').perform(context)
    yaw_spawn = LaunchConfiguration('yaw_spawn').perform(context)

    
    robot_description_topic_name = "/" + robot_name + "/robot_description"
    joint_state_topic_name = "/" + robot_name + "/joint_states"
    robot_state_publisher_name =  robot_name + "_robot_state_publisher"
    # robot_state_publisher_name = "robot_state_publisher"

    package_description = "journal1-sim"    

    extension = robot_file.split(".")[2]

    if extension == "urdf":
        robot_desc_path = os.path.join(get_package_share_directory(
        package_description), "urdf", robot_file)
        robot_desc = xacro.process_file(robot_desc_path)
    elif extension == "xacro":
        robot_desc_path = os.path.join(get_package_share_directory(
        package_description), "robot", robot_file)
        robot_desc = xacro.process_file(robot_desc_path, mappings={'robot_name' : robot_name})
    else:
        assert False, "Extension of robot file not suppored = "+str(extension)

 
    xml = robot_desc.toxml()
    print(f"xml type is {type(xml)}")
    print(f"xml is {xml}")
    

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': xml}], #, 'frame_prefix': robot_name + '/'}],
        remappings=[# ("/robot_description", robot_description_topic_name),
        #             ("/joint_states", joint_state_topic_name)
                    ("/tf", "tf"), ("/tf_static", "tf_static")], 
        namespace=robot_name,  # Added namespace to the node
        output="screen"
    )
    
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='robot_spawn_entity_' + robot_name,
        output='screen',
        emulate_tty=True,
        arguments=['-entity',robot_name,
                    '-x', str(x_spawn), '-y', str(y_spawn), '-z', str(z_spawn),
                    '-R', str(roll_spawn), '-P', str(pitch_spawn), '-Y', str(yaw_spawn),
                    '-topic', robot_description_topic_name,
                    '-robot_namespace', '/' + robot_name,  # Added leading slash to namespace
                    # '-gazebo_namespace', robot_name
                    ],
        # remappings=[("/robot_state_publisher", robot_state_publisher_name)
        #             ],
        namespace=robot_name  # Added namespace to the node
    )
  
    return [robot_state_publisher_node, spawn_robot]

def generate_launch_description(): 

    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='robot')
    robot_file_arg = DeclareLaunchArgument('robot_file', default_value='robot.urdf')
    
    x_spawn_arg = DeclareLaunchArgument('x_spawn', default_value='0.0')
    y_spawn_arg = DeclareLaunchArgument('y_spawn', default_value='0.0')
    z_spawn_arg = DeclareLaunchArgument('z_spawn', default_value='0.0')
    roll_spawn_arg = DeclareLaunchArgument('roll_spawn', default_value='0.0')
    pitch_spawn_arg = DeclareLaunchArgument('pitch_spawn', default_value='0.0')
    yaw_spawn_arg = DeclareLaunchArgument('yaw_spawn', default_value='0.0')
    
    return LaunchDescription([
        robot_name_arg,
        robot_file_arg,
        
        x_spawn_arg,
        y_spawn_arg, 
        z_spawn_arg,
        roll_spawn_arg,
        pitch_spawn_arg,
        yaw_spawn_arg,
        OpaqueFunction(function = launch_setup)
        ])