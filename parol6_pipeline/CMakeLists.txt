cmake_minimum_required(VERSION 3.8)
project(parol6_pipeline)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =====================================================
#                  FIND REQUIRED PACKAGES
# =====================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(msg_gazebo REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(SQLite3 REQUIRED)

# =====================================================
#                  INCLUDE DIRECTORIES
# =====================================================
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
)

# =====================================================
#              ARUCO POSE DETECTOR NODE
# =====================================================
add_executable(aruco_pose_detector
  src/aruco_pose_detector.cpp
)

ament_target_dependencies(aruco_pose_detector
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  image_transport
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
  moveit_ros_planning_interface
  OpenCV
)

target_link_libraries(aruco_pose_detector
  ${OpenCV_LIBS}
)

# =====================================================
#              PICK CONTROLLER NODE
# =====================================================
add_executable(pick_controller
  src/pick_controller.cpp
)

ament_target_dependencies(pick_controller
  rclcpp
  geometry_msgs
  std_msgs
  std_srvs
  moveit_ros_planning_interface
  moveit_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# =====================================================
#              INSTALL TARGETS
# =====================================================
install(TARGETS
  aruco_pose_detector
  pick_controller  # <-- CRUCIAL FIX: Matches the 'add_executable' line
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()