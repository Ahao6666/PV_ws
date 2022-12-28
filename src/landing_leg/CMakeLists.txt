cmake_minimum_required(VERSION 2.8.3)
project(landing_leg)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  catkin_simple
  roscpp
  rospy
  std_msgs
  message_generation
  actionlib_msgs
  trajectory_msgs
  gazebo_msgs
)
find_package(gazebo REQUIRED)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  kpkv_msg.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  traj.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  actionlib_msgs
  trajectory_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES landing_leg
  CATKIN_DEPENDS actionlib catkin_simple roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(landing_leg_controller src/landing_leg_controller.cpp)
add_executable(landing_leg_kpkv_serviceClient src/kpkv_serviceClient.cpp)
add_executable(landing_leg_server src/landing_leg_server.cpp)
add_executable(landing_leg_client src/landing_leg_client.cpp)
add_executable(landing_leg_client_init src/landing_leg_client_init.cpp)
add_executable(landing_leg_tcp_client src/landing_leg_tcp_client.cpp)
add_executable(landing_leg_tcp_server src/landing_leg_tcp_server.cpp)
add_executable(force_torque_sensor src/force_torque_sensor.cpp)
# add_library(force_torque_sensor src/force_torque_sensor.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(landing_leg_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(landing_leg_controller landing_leg_gencpp)
add_dependencies(landing_leg_kpkv_serviceClient landing_leg_gencpp)
add_dependencies(landing_leg_server landing_leg_gencpp)
add_dependencies(landing_leg_client landing_leg_gencpp)
add_dependencies(landing_leg_client_init landing_leg_gencpp)
add_dependencies(landing_leg_tcp_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(landing_leg_tcp_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(force_torque_sensor landing_leg_gencpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(landing_leg_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(landing_leg_controller ${catkin_LIBRARIES})
target_link_libraries(landing_leg_kpkv_serviceClient ${catkin_LIBRARIES})
target_link_libraries(landing_leg_server ${catkin_LIBRARIES})
target_link_libraries(landing_leg_client ${catkin_LIBRARIES})
target_link_libraries(landing_leg_client_init ${catkin_LIBRARIES})
target_link_libraries(landing_leg_tcp_client ${catkin_LIBRARIES})
target_link_libraries(landing_leg_tcp_server ${catkin_LIBRARIES})
target_link_libraries(force_torque_sensor ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
