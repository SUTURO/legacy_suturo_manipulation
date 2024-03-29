cmake_minimum_required(VERSION 2.8.3)
project(suturo_manipulation_action)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  moveit_ros_planning_interface
  roscpp
  rospy
  suturo_manipulation_msgs
  actionlib 
  actionlib_msgs
  tf
  suturo_manipulation_move_robot
  suturo_manipulation_grasping
  suturo_manipulation_planning_scene_interface
  suturo_manipulation_gripper_controller
  suturo_manipulation_tools
  suturo_manipulation_graspposition_calculation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

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
#   suturo_perception_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES suturo_manipulation_action
#  CATKIN_DEPENDS message_generation moveit_ros_planning_interface roscpp rospy suturo_manipulation_msgs/
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
#include_directories(
#  ${catkin_INCLUDE_DIRS}
#)

## Declare a cpp library
# add_library(suturo_manipulation_action
#   src/${PROJECT_NAME}/suturo_manipulation_action.cpp
# )

## Declare a cpp executable
# add_executable(suturo_manipulation_action_node src/suturo_manipulation_action_node.cpp)


add_executable(suturo_manipulation_move_home_server src/suturo_manipulation_move_home_server.cpp)
add_executable(suturo_manipulation_base_server src/suturo_manipulation_base_server.cpp)
add_executable(suturo_manipulation_move_home_server_ros src/suturo_manipulation_move_home_server_ros.cpp)
add_executable(suturo_manipulation_move_arm_server src/suturo_manipulation_move_arm_server.cpp)
add_executable(suturo_manipulation_grasping_action_server src/suturo_manipulation_grasping_action_server.cpp)
add_executable(test_move_home_client src/test_move_home_client.cpp)
add_executable(test_grasping_client src/test_grasping_client.cpp)
add_executable(test_move_arm_client src/test_move_arm_client.cpp)
add_executable(suturo_manipulation_move_head_server src/suturo_manipulation_move_head_server.cpp)
add_executable(suturo_manipulation_move_head_server_ros src/suturo_manipulation_move_head_server_ros.cpp)
add_executable(test_head_client src/test_head_client.cpp)
add_executable(test_move_base_client src/test_move_base_client.cpp)
add_executable(test_head_subscriber src/test_head_subscriber.cpp)
add_executable(head_mover src/head_mover.cpp)
add_executable(collision_tester src/collision_tester.cpp)
add_executable(spawn_planningscene src/spawn_planningscene.cpp)
add_executable(spawn_planningscene_gazebo src/spawn_planningscene_gazebo.cpp)



## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(suturo_manipulation_action_node suturo_manipulation_action_generate_messages_cpp)
add_dependencies(suturo_manipulation_move_home_server suturo_manipulation_msgs ${catkin_EXPORTED_TARGETS})
add_dependencies(suturo_manipulation_base_server suturo_manipulation_msgs suturo_manipulation_move_robot ${catkin_EXPORTED_TARGETS})
add_dependencies(suturo_manipulation_move_home_server_ros suturo_manipulation_planning_scene_interface suturo_manipulation_msgs ${catkin_EXPORTED_TARGETS})
add_dependencies(suturo_manipulation_move_arm_server suturo_manipulation_msgs suturo_manipulation_planning_scene_interface suturo_manipulation_grasping ${catkin_EXPORTED_TARGETS})
add_dependencies(suturo_manipulation_grasping_action_server suturo_manipulation_msgs suturo_manipulation_gripper_controller suturo_manipulation_planning_scene_interface suturo_manipulation_grasping suturo_manipulation_grasping_reactive ${catkin_EXPORTED_TARGETS})
add_dependencies(test_move_home_client suturo_manipulation_msgs ${catkin_EXPORTED_TARGETS})
add_dependencies(test_move_arm_client suturo_manipulation_msgs ${catkin_EXPORTED_TARGETS})
add_dependencies(test_grasping_client suturo_manipulation_msgs ${catkin_EXPORTED_TARGETS})


add_dependencies(head_mover suturo_manipulation_move_robot suturo_manipulation_msgs suturo_manipulation_gripper_controller suturo_manipulation_planning_scene_interface suturo_manipulation_mesh_loader suturo_manipulation_grasping suturo_manipulation_grasping_reactive ${catkin_EXPORTED_TARGETS})

add_dependencies(collision_tester suturo_manipulation_move_robot suturo_manipulation_msgs suturo_manipulation_gripper_controller suturo_manipulation_planning_scene_interface suturo_manipulation_mesh_loader suturo_manipulation_grasping suturo_manipulation_grasping_reactive ${catkin_EXPORTED_TARGETS})

add_dependencies(suturo_manipulation_move_head_server suturo_manipulation_msgs ${catkin_EXPORTED_TARGETS})
add_dependencies(suturo_manipulation_move_head_server_ros suturo_manipulation_msgs ${catkin_EXPORTED_TARGETS})
add_dependencies(test_head_client suturo_manipulation_msgs ${catkin_EXPORTED_TARGETS})
add_dependencies(test_move_base_client suturo_manipulation_msgs suturo_manipulation_move_robot ${catkin_EXPORTED_TARGETS})
add_dependencies(test_head_subscriber suturo_manipulation_msgs ${catkin_EXPORTED_TARGETS})
add_dependencies(spawn_planningscene suturo_manipulation_msgs ${catkin_EXPORTED_TARGETS})
add_dependencies(spawn_planningscene_gazebo suturo_manipulation_mesh_loader suturo_manipulation_msgs ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
# target_link_libraries(suturo_manipulation_action_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(
  suturo_manipulation_move_home_server
  ${catkin_LIBRARIES}
)

target_link_libraries(
  suturo_manipulation_base_server
  suturo_manipulation_move_robot
  ${catkin_LIBRARIES}
)

target_link_libraries(
  suturo_manipulation_move_home_server_ros
  suturo_manipulation_planning_scene_interface
  ${catkin_LIBRARIES}
)

target_link_libraries(
  suturo_manipulation_move_arm_server
  suturo_manipulation_planning_scene_interface
  suturo_manipulation_grasping
  ${catkin_LIBRARIES}
)

target_link_libraries(
  spawn_planningscene
  ${catkin_LIBRARIES}
)

target_link_libraries(
  spawn_planningscene_gazebo
suturo_manipulation_mesh_loader
  ${catkin_LIBRARIES}
)

target_link_libraries(
  suturo_manipulation_grasping_action_server
  suturo_manipulation_gripper_controller
  suturo_manipulation_planning_scene_interface
  suturo_manipulation_grasping
  suturo_manipulation_grasping_reactive
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(
  test_move_home_client
  ${catkin_LIBRARIES}
)

target_link_libraries(
  test_move_arm_client
  ${catkin_LIBRARIES}
)

target_link_libraries(
  test_grasping_client
  ${catkin_LIBRARIES}
)

target_link_libraries(
  suturo_manipulation_move_head_server
  ${catkin_LIBRARIES}
)

target_link_libraries(
  suturo_manipulation_move_head_server_ros
  ${catkin_LIBRARIES}
)

target_link_libraries(
  test_head_client
  ${catkin_LIBRARIES}
)

target_link_libraries(
  test_head_subscriber
  ${catkin_LIBRARIES}
)

target_link_libraries(
  test_move_base_client
  suturo_manipulation_move_robot
  ${catkin_LIBRARIES}
)

target_link_libraries(
  head_mover
  suturo_manipulation_gripper_controller
  suturo_manipulation_planning_scene_interface
  suturo_manipulation_grasping
  suturo_manipulation_move_robot
  suturo_manipulation_grasping_reactive
  suturo_manipulation_mesh_loader
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(
  collision_tester
  suturo_manipulation_gripper_controller
  suturo_manipulation_planning_scene_interface
  suturo_manipulation_grasping
  suturo_manipulation_move_robot
  suturo_manipulation_grasping_reactive
  suturo_manipulation_mesh_loader
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS suturo_manipulation_action suturo_manipulation_action_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


#install(TARGETS test DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_suturo_manipulation_action.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
