cmake_minimum_required(VERSION 2.8.3)
project(gbx_kf)

#NOTE: Eigen includes are different in the snapdragon and odroid.  Snap uses Eigen includes, Odroid uses Eigen3 includes.

#list(APPEND CMAKE_INCLUDE_PATH "/opt/ros/kinetic/share/cmake_modules/cmake/Modules/")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  tf2_ros
  cmake_modules
  nav_msgs
  gbx_ros_bridge_msgs
  message_generation
)

if(NOT GBX_VERSION)
    set(GBX_VERSION 4)
endif()


set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(Eigen REQUIRED)
#find_package (Eigen3 3.3 REQUIRED NO_MODULE)

#catkin_package(
#    INCLUDE_DIRS include
#    CATKIN_DEPENDS gbx_ros_bridge_msgs
#    DEPENDS Eigen
#)

# # NOTE: USES GPS_KF MESSAGES SO INSTALLING THIS AS GPSKF_GBX IS FINE.
# #   DOING SO ADDRESSES A CMAKE/PYTHON MISMATCH
#add_message_files(
#  FILES
#  twUpdate.msg
#  odomWithGpsTime.msg
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  geometry_msgs
#  gbx_ros_bridge_msgs
#  nav_msgs
#)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
#  ${Eigen3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS std_msgs geometry_msgs message_runtime gbx_ros_bridge_msgs)

add_executable(gpskf_gbx src/main.cpp src/gps_odom.cpp src/filter.cpp src/filterTW.cpp src/transformations.cpp src/gpsCallbacks.cpp src/viconCallback.cpp src/gbxStreamEndpointGPSKF.cpp
  include/filter.h include/filterTW.h include/gbxStreamEndpointGPSKF.hpp include/gpsTime.h include/gps_odom.hpp include/transformations.hpp )

target_link_libraries(gpskf_gbx 
  SharedPpEngine
  Threads::Threads
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GPSTK_LIBRARIES}
  ${Eigen_LIBRARIES}
  )
#target_link_libraries(gpskf_gbx Eigen3::Eigen)

set_property(TARGET gpskf_gbx PROPERTY CXX_STANDARD 11)
set_property(TARGET gpskf_gbx PROPERTY CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

install(FILES
  launch/gpsGbx.launch
#   # myfile1
#   # myfile2
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


