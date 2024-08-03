cmake_minimum_required(VERSION 3.8)
project(custom_messages)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(autoware_auto_perception_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_messages REQUIRED)
find_package(PCL 1.12 REQUIRED)
find_package(pcl_conversions REQUIRED)

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

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedObject.msg"
  "msg/DetectedObjects.msg"
  DEPENDENCIES sensor_msgs autoware_auto_perception_msgs
)
ament_export_dependencies(rosidl_default_runtime)

add_executable(detected_objects_visualizer_node
  src/detected_objects_visualizer.cpp
)

ament_target_dependencies(detected_objects_visualizer_node rclcpp sensor_msgs autoware_auto_perception_msgs pcl_conversions custom_messages)
target_link_libraries(detected_objects_visualizer_node ${PCL_LIBRARIES})

install(TARGETS
  detected_objects_visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
