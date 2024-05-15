import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.substitutions import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro




def generate_launch_description():
  
    share_dir_path = os.path.join(get_package_share_directory('journal1-sim'))
    xacro_path = os.path.join(share_dir_path, 'urdf', 'smagv', 'smagv.urdf.xacro')
    urdf_path = os.path.join(share_dir_path, 'urdf', 'smagv', 'smagv.urdf')

    # xacroをロード
    doc = xacro.process_file(xacro_path)
    # xacroを展開してURDFを生成
    robot_desc = doc.toprettyxml(indent='  ')
    # urdf_pathに対してurdfを書き出し
    f = open(urdf_path, 'w')
    f.write(robot_desc)
    f.close()
    rsp = Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='both',
                                  # argumentsでURDFを出力したパスを指定
                                  arguments=[urdf_path])
    
    joint_state_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        )
    
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    rviz_group = GroupAction(
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

    return LaunchDescription([rsp, joint_state_publisher, joint_state_publisher_gui, rviz_group])
