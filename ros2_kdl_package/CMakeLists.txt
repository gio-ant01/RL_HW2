cmake_minimum_required(VERSION 3.8)
project(ros2_kdl_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2 REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

#################### server #######################
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
#################### client #######################
find_package(rclcpp_components REQUIRED)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/LinearTrajectory.action"
)

# Example node (core)
add_executable(ros2_kdl_node src/ros2_kdl_node.cpp src/kdl_robot.cpp src/kdl_planner.cpp src/kdl_control.cpp)
target_include_directories(ros2_kdl_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})
target_compile_features(ros2_kdl_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(ros2_kdl_node 
  orocos_kdl
  rclcpp
  kdl_parser
  sensor_msgs
  std_msgs
  urdf
  urdfdom_headers)

install(TARGETS ros2_kdl_node
  DESTINATION lib/${PROJECT_NAME}
)

######################## server #################################
add_executable(ros2_kdl_action_server src/ros2_kdl_action_server.cpp src/kdl_robot.cpp src/kdl_planner.cpp src/kdl_control.cpp)
target_include_directories(ros2_kdl_action_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})
target_compile_features(ros2_kdl_action_server PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(ros2_kdl_action_server 
  orocos_kdl
  rclcpp
  rclcpp_action
  kdl_parser
  sensor_msgs
  std_msgs
  urdf
  urdfdom_headers)

# Make sure the action is generated before building the server
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(ros2_kdl_action_server "${cpp_typesupport_target}")

install(TARGETS ros2_kdl_action_server
  DESTINATION lib/${PROJECT_NAME}
)

###################### client ##########################
add_library(action_client SHARED
  src/ros2_kdl_action_client.cpp)
target_include_directories(action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(action_client PUBLIC cxx_std_17)

ament_target_dependencies(action_client
  rclcpp
  rclcpp_action
  rclcpp_components
  urdf
  urdfdom_headers)

# Make sure the action is generated before building the client
target_link_libraries(action_client "${cpp_typesupport_target}")

rclcpp_components_register_node(action_client 
  PLUGIN "ros2_kdl_package::LinearTrajectoryActionClient" 
  EXECUTABLE ros2_kdl_action_client)

install(TARGETS
  action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Example node
add_executable(ros2_kdl_node_example src/ros2_kdl_node_example.cpp)
target_include_directories(ros2_kdl_node_example PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})
target_compile_features(ros2_kdl_node_example PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(ros2_kdl_node_example 
  orocos_kdl
  rclcpp)

install(TARGETS ros2_kdl_node_example
  DESTINATION lib/${PROJECT_NAME})

# Vision-based control node
add_executable(ros2_kdl_vision_node 
  src/ros2_kdl_vision_node.cpp 
  src/kdl_robot.cpp 
  src/kdl_control.cpp)
target_include_directories(ros2_kdl_vision_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})
target_compile_features(ros2_kdl_vision_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(ros2_kdl_vision_node 
  orocos_kdl
  rclcpp
  kdl_parser
  tf2
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs)

install(TARGETS ros2_kdl_vision_node
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(ros2_kdl_vision_node
    orocos-kdl
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()