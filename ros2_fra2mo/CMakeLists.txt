cmake_minimum_required(VERSION 3.5)
project(ros2_fra2mo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# ================================================================
# DIPENDENZE
# ================================================================

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# ================================================================
# NODO TF
# ================================================================

add_executable(dynamic_tf_publisher
  src/odom_bl_tf.cpp
)

ament_target_dependencies(
  dynamic_tf_publisher
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

# ================================================================
# NODO CONTROLLO ROVER
# ================================================================

add_executable(fra2mo_controller_node
  src/fra2mo_controller_node.cpp
)

ament_target_dependencies(
  fra2mo_controller_node
  rclcpp
  geometry_msgs
  nav_msgs
)

# ================================================================
# INSTALL
# ================================================================

install(
  DIRECTORY
    launch
    urdf
    models
    meshes
    worlds
    config
    src
    maps
    scripts
    rviz_conf
  DESTINATION
    share/${PROJECT_NAME}
)

# ================================================================
# INSTALL NODI
# ================================================================

install(
  TARGETS
    dynamic_tf_publisher
    fra2mo_controller_node
  DESTINATION
    lib/${PROJECT_NAME}
)

# ================================================================
# PYTHON EXECUTABLES
# ================================================================

install(
  PROGRAMS
    scripts/reach_goal.py
    scripts/follow_waypoints.py
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
