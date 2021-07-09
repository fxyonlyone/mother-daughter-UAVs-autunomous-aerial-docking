cmake_minimum_required(VERSION 3.0.2)
project(muav_move)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  roscpp
  rospy
  std_msgs
  duav_dock
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
 CATKIN_DEPENDS
  geometry_msgs
  mavros
  roscpp
  rospy
  std_msgs
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

#############
## Testing ##
#############

add_executable(mUAVTakeOff src/mUAVTakeOff.cpp)
target_link_libraries(mUAVTakeOff
${catkin_LIBRARIES}
)

