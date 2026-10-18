cmake_minimum_required(VERSION 3.8)
project(hospital)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Dipendenze C++ e ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)

# 2. Compilazione dei nodi C++

# --- IIWA2 PICK NODE ---
add_executable(iiwa2_pick_kdl_node src/iiwa2_pick_kdl_node.cpp)
ament_target_dependencies(iiwa2_pick_kdl_node
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  kdl_parser
  orocos_kdl_vendor
  urdf
)
target_link_libraries(iiwa2_pick_kdl_node
  Eigen3::Eigen
)

# --- IIWA1 PICK NODE ---
add_executable(iiwa1_pick_node src/iiwa1_pick_node.cpp)
ament_target_dependencies(iiwa1_pick_node
  rclcpp
  std_msgs
  sensor_msgs
  kdl_parser
  orocos_kdl_vendor
)
target_include_directories(iiwa1_pick_node PRIVATE
  ${Eigen3_INCLUDE_DIRS}
)
target_link_libraries(iiwa1_pick_node
  ${Eigen3_LIBRARIES}
)


# 3. Installazione degli eseguibili C++
install(TARGETS
  iiwa2_pick_kdl_node
  iiwa1_pick_node
  DESTINATION lib/${PROJECT_NAME}
)

# 4. Installazione degli script Python esistenti
install(
  PROGRAMS
    scripts/user_request_node.py
    scripts/object_detector.py
    scripts/aruco_docking_node.py
    scripts/task_coordinator.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
