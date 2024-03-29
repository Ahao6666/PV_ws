cmake_minimum_required(VERSION 2.8.3)
project(four_landing_leg)

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
## Generate message in the 'msg' folder
add_message_files(
  FILES
  four_landing_leg_cmd.msg
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
#  LIBRARIES four_landing_leg
  CATKIN_DEPENDS actionlib catkin_simple roscpp rospy std_msgs message_runtime
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
add_executable(landing_leg_ft_sensor src/landing_leg_ft_sensor.cpp)
add_executable(force_torque_sensor src/force_torque_sensor.cpp)
# add_executable(velodyne_distance_sensor src/velodyne_distance_sensor.cpp)
add_executable(pv_car_pose_pub src/pv_car_pose_pub.cpp)
add_executable(test src/test.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(landing_leg_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(landing_leg_controller four_landing_leg_gencpp)
add_dependencies(landing_leg_kpkv_serviceClient four_landing_leg_gencpp)
add_dependencies(landing_leg_server four_landing_leg_gencpp)
add_dependencies(landing_leg_client four_landing_leg_gencpp)
add_dependencies(landing_leg_client_init four_landing_leg_gencpp)
add_dependencies(landing_leg_tcp_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(landing_leg_tcp_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(landing_leg_ft_sensor four_landing_leg_gencpp)
add_dependencies(force_torque_sensor four_landing_leg_gencpp)
# add_dependencies(velodyne_distance_sensor four_landing_leg_gencpp)
add_dependencies(pv_car_pose_pub four_landing_leg_gencpp)
add_dependencies(test four_landing_leg_gencpp)

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
target_link_libraries(landing_leg_ft_sensor ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(force_torque_sensor ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
# target_link_libraries(velodyne_distance_sensor ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(pv_car_pose_pub ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(test ${catkin_LIBRARIES})
