cmake_minimum_required(VERSION 3.0.2)
project(sofar_assignment)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  trajectory_msgs
  actionlib_msgs
  actionlib
  gazebo_msgs
    
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   Spawner.srv
   DetectColor.srv
   Conveyor.srv
 )
## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs   gazebo_msgs#sensor_msgs#   std_msgs#   trajectory_msgs
 )
###########
## Build ##
###########
catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/sofar_assignment.cpp
# )


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
 add_executable(move_head_node src/move_head.cpp)
 add_executable(main_node src/main.cpp)
 #add_executable(prova_node src/prova.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(move_head_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(main_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 # add_dependencies(prova_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(move_head_node
   ${catkin_LIBRARIES}
 )
 target_link_libraries(main_node
   ${catkin_LIBRARIES}
  )
  #target_link_libraries(prova_node
    #${catkin_LIBRARIES}
  #)
 #target_link_libraries(color_detection_node
 #${catkin_LIBRARIES}
#)


#############
## Install ##
#############


