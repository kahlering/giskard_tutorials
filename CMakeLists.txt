cmake_minimum_required(VERSION 2.8.3)
project(giskard_tutorials)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11 -g)

#catkin_make -DCMAKE_BUILD_TYPE=Debug


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  giskard_msgs
  actionlib_msgs
  genmsg
  actionlib
)

add_action_files(DIRECTORY /../giskard_msgs/action FILES ControllerList.action)
generate_messages(DEPENDENCIES actionlib_msgs giskard_msgs)


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/giskard_tutorials.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
 add_executable(joint_space_action_client src/joint_space_action_client.cpp)
 add_executable(cartesian_space_action_client src/cartesian_space_action_client.cpp)
 add_executable(joint_and_cartesian_space_commands src/joint_and_cartesian_space_commands.cpp)



## Specify libraries to link a library or executable target against

target_link_libraries(cartesian_space_action_client ${catkin_LIBRARIES})
target_link_libraries(joint_space_action_client ${catkin_LIBRARIES})
target_link_libraries(joint_and_cartesian_space_commands ${catkin_LIBRARIES})