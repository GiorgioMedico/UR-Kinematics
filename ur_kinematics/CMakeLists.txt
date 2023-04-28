cmake_minimum_required(VERSION 3.0.2)
project(ur_kinematics)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  message_generation
  actionlib
  actionlib_msgs
  tf
  kdl_parser
  )
  
find_package(orocos_kdl REQUIRED)
find_package( Eigen3 REQUIRED )
  

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  JointMatrix.msg
)

add_service_files(
  FILES
  UrForwardKinematics.srv
  UrInverseKinematics.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  actionlib_msgs
  std_srvs
)

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES ur_kinematics
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs std_srvs
  DEPENDS 
)


include_directories(
  include
  include/ur_kinematics
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/ik_ur.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ur_kinematics_main src/ur_kinematics.cpp)
target_link_libraries(ur_kinematics_main ${catkin_LIBRARIES} ur_kinematics)
add_dependencies(ur_kinematics_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

