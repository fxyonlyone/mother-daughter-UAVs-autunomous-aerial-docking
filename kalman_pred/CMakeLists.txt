cmake_minimum_required(VERSION 3.0.2)
project(kalman_pred)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  dUAV_dock
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS thread)
find_library(NEWMAT newmat /usr/lib)


###################################
## catkin specific configuration ##
###################################
catkin_package(
 CATKIN_DEPENDS
  roscpp
  std_msgs
)

set(HEADER_FILES
    include/kalman_xy_pos.hpp

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${NEWMAT_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})



## Declare a cpp library
add_executable(kalman_xy_pos
               src/kalman_xy_pos.cpp
               ${HEADER_FILES}
)

## Specify libraries to link a library or executable target against
target_link_libraries(kalman_xy_pos ${NEWMAT})
target_link_libraries(kalman_xy_pos ${catkin_LIBRARIES})
