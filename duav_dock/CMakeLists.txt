cmake_minimum_required(VERSION 2.8.3)
project(duav_dock)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  mavros
  roscpp
  rospy
  std_msgs
  
)
add_message_files(FILES 
Center.msg If_dUAV_docked.msg Marker_find_info.msg Kalman_info.msg Track_state.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
CATKIN_DEPENDS 
geometry_msgs 
mavros 
roscpp 
rospy 
std_msgs
message_runtime

)

###########
## Build ##
###########
include_directories(include)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Testing ##
#############

add_executable(dUAVDock src/dUAVDock.cpp)
target_link_libraries(dUAVDock
${catkin_LIBRARIES}
)
