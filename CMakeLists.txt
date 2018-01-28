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



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
 add_executable(joint_space_action_client_example src/examples/joint_space_action_client.cpp)
 add_executable(cartesian_space_action_client_example src/examples/cartesian_space_action_client.cpp)
 add_executable(joint_and_cartesian_space_action_client_example src/examples/joint_and_cartesian_space_commands.cpp)



## Specify libraries to link a library or executable target against

target_link_libraries(joint_space_action_client_example ${catkin_LIBRARIES})
target_link_libraries(cartesian_space_action_client_example ${catkin_LIBRARIES})
target_link_libraries(joint_and_cartesian_space_action_client_example ${catkin_LIBRARIES})