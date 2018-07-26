cmake_minimum_required(VERSION 2.8.3)
project(gps_kf_hardwired)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  tf2_ros
  cmake_modules
  nav_msgs
  gbx_ros_bridge_msgs
  px4_control
)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
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
  ${Boost_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS std_msgs geometry_msgs message_runtime gbx_ros_bridge_msgs
    DEPENDS Eigen)

add_executable(gpsKf_node src/gps_odom.cpp src/filter.cpp src/filterTW.cpp src/transformations.cpp src/gpsCallbacks.cpp src/viconCallback.cpp)

target_link_libraries(gpsKf_node 
  SharedPpEngine
  Threads::Threads
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GPSTK_LIBRARIES}  )

set_property(TARGET gpsKf_node PROPERTY CXX_STANDARD 11)
set_property(TARGET gpsKf_node PROPERTY CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

#add_executable(gpsKf_node src/gps_odom.cpp src/filter.cpp src/filterTW.cpp src/transformations.cpp src/gpsCallbacks.cpp src/viconCallback.cpp)
add_dependencies(gpsKf_node gbx_ros_bridge_msgs_gencpp)
#add_dependencies(gpsKf_node gbx_ros_bridge_msgs)
#add_dependencies(gpsKf_node gps_kf_generate_messages_cpp)
#target_link_libraries(gpsKf_node ${catkin_LIBRARIES})

install(FILES
  gpsQuad.launch
#   # myfile1
#   # myfile2
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


