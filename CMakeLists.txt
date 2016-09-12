cmake_minimum_required(VERSION 2.8.3)
project(workspace_transformation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED roscpp rospy geometry_msgs std_msgs message_generation)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and scripts declared therein get installed
# catkin_python_setup()


#######################################
## Declare ROS messages and services ##
#######################################

# Generate services in the 'srv' folder
add_service_files(
  DIRECTORY srv
  FILES
  setAlgorithm.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)



###################################################
## Declare things to be passed to other projects ##
###################################################

catkin_package(
 INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}
#  LIBRARIES workspace_transformation_tube
 DEPENDS libeigen3-dev
 CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)
INCLUDE_DIRECTORIES(include ${catkin_INCLUDE_DIRS})
include_directories(/usr/include/eigen3)



###########
## Build ##
###########

add_executable(workspace_transformation src/workspace_transformation_node_v2.cpp)
add_dependencies(workspace_transformation  ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(workspace_transformation ${catkin_LIBRARIES})

## Workspace with free flying camera
add_executable(workspaceTransformationCam src/workspace_transformation_cam_node.cpp)
add_dependencies(workspaceTransformationCam ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(workspaceTransformationCam ${catkin_LIBRARIES})

## Workspace with free flying camera for real implementation
add_executable(workspaceTransformationCamReal src/workspace_transformation_cam_node_real.cpp)
add_dependencies(workspaceTransformationCamReal ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(workspaceTransformationCamReal ${catkin_LIBRARIES})



###########
## Install ##
###########

set(WOMAP_TARGETS workspaceTransformationCam workspace_transformation workspaceTransformationCamReal)

install(TARGETS ${WOMAP_TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)