cmake_minimum_required(VERSION 2.8.3)
project(gps_kf)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  tf2_ros
  cmake_modules
  nav_msgs
  gbx_ros_bridge_msgs
  px4_control
)

find_package(Eigen REQUIRED)

#catkin_package(
#    INCLUDE_DIRS include
#    CATKIN_DEPENDS gbx_ros_bridge_msgs
#    DEPENDS Eigen
#)

add_message_files(
  FILES
  twUpdate.msg
  odomWithGpsTime.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  gbx_ros_bridge_msgs
  nav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS std_msgs geometry_msgs message_runtime gbx_ros_bridge_msgs
    DEPENDS Eigen)

add_executable(gpsKf_node src/gps_odom.cpp src/filter.cpp src/filterTW.cpp src/transformations.cpp src/gpsCallbacks.cpp src/viconCallback.cpp)
add_dependencies(gpsKf_node gbx_ros_bridge_msgs_gencpp)
add_dependencies(gpsKf_node gbx_ros_bridge_msgs)
add_dependencies(gpsKf_node gps_kf_generate_messages_cpp)
target_link_libraries(gpsKf_node ${catkin_LIBRARIES})

