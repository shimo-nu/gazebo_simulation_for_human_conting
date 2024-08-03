cmake_minimum_required(VERSION 3.8)
project(journal1-sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_PREFIX_PATH "/usr/include/pcl-1.12")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(PCL 1.12 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(custom_messages)

find_package(autoware_auto_perception_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(${PROJECT_NAME}
  src/joy_twist_publisher.cpp
)

add_executable(receive_detected_object_node
  src/receive_detected_objects.cpp
)

add_executable(visualize_marker_of_objects_node
  src/visualize_marker_of_objects.cpp
)

add_executable(odometry_twist_converter_node
  src/odometry_twist_converter.cpp
)

add_executable(multi_robot_object_detection_merger_node
  src/multi_robot_object_detection_merger.cpp
)

add_executable(check_detected_objects_node
  src/check_detected_objects.cpp
)

# add_executable(human_detection_node
#   src/human_detection.cpp
# )

ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs geometry_msgs)

ament_target_dependencies(receive_detected_object_node rclcpp autoware_auto_perception_msgs sensor_msgs geometry_msgs)

ament_target_dependencies(visualize_marker_of_objects_node rclcpp autoware_auto_perception_msgs sensor_msgs visualization_msgs)

ament_target_dependencies(odometry_twist_converter_node rclcpp nav_msgs geometry_msgs)

ament_target_dependencies(multi_robot_object_detection_merger_node rclcpp pcl_conversions autoware_auto_perception_msgs custom_messages)
target_link_libraries(multi_robot_object_detection_merger_node ${PCL_LIBRARIES})

ament_target_dependencies(check_detected_objects_node rclcpp pcl_conversions sensor_msgs custom_messages)
target_link_libraries(check_detected_objects_node ${PCL_LIBRARIES})

# ament_target_dependencies(human_detection_node rclcpp sensor_msgs pcl_conversions pcl_ros tf2_eigen geometry_msgs)

# target_link_libraries(human_detection_node ${PCL_LIBRARIES})



install(TARGETS
  ${PROJECT_NAME}
  receive_detected_object_node
  visualize_marker_of_objects_node
  odometry_twist_converter_node
#  human_detection_node
  multi_robot_object_detection_merger_node
  check_detected_objects_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  config
  world
  urdf
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
