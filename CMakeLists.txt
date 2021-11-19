cmake_minimum_required(VERSION 2.8.3)
project(robot_dock_task)

# we are using C++11
SET(CMAKE_CXX_FLAGS "-std=c++0x")

# set if using ROS
add_definitions(-DUSING_ROS)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  message_generation
  actionlib
  actionlib_msgs
  robot_docker
  # dynamic reconfigure
  #dynamic_reconfigure
  #rospy
)

################################################
## Declare ROS messages, services and actions ##
################################################
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

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################
## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES task_handler_template
  CATKIN_DEPENDS roscpp roslib std_msgs actionlib_msgs
  robot_docker
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(robot_dock_task_node        src/main_node.cpp
                                            src/base.hpp
                                            src/task_handler.cpp
                                            ../../mrccc/common/mission/basedata.cpp
                                            ../../mrccc/common/mission/tasks_definition.cpp
                                            ../../mrccc/common/mission/task_definition_data.cpp
                                            ../../mrccc/common/utils/utils_tasks_definition.hpp
                                            ../../mrccc/common/logger.cpp
                                            ../../mrccc/common/logger4cxx.cpp
)

## Add cmake target dependencies of the executable
add_dependencies(robot_dock_task_node  ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                        ${catkin_EXPORTED_TARGETS}
                                        robot_docker
)

## Specify libraries to link a library or executable target against
target_link_libraries(robot_dock_task_node
  ${catkin_LIBRARIES}
  jsoncpp
)

#############
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
# install(TARGETS task_handler_template_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
## if you want finer control, the do file by file as follows
# install(FILES
#   launch/start.launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )
