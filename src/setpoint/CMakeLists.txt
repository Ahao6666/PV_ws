cmake_minimum_required(VERSION 3.0.2)
project(pv_mission)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  roscpp
  cmake_modules
  landing_leg
)

find_package(Eigen3 REQUIRED) # 添加 2
  catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pv_mission
  CATKIN_DEPENDS geometry_msgs mavros roscpp
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( include ${catkin_INCLUDE_DIRS})
include_directories( ${/usr/local/include/eigen3/Eigen}) # 添加3


 add_executable(off_mission_node src/offboard_node.cpp)
 add_executable(actuator_ctrl src/actuator_ctrl.cpp)
 add_executable(att_rate_setpoint src/att_rate_setpoint.cpp)
 add_executable(att_setpoint src/att_setpoint.cpp)
 add_executable(inclined_landing src/inclined_landing.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(off_mission_node   ${catkin_LIBRARIES} )
 target_link_libraries(actuator_ctrl   ${catkin_LIBRARIES} )
 target_link_libraries(att_rate_setpoint   ${catkin_LIBRARIES} )
 target_link_libraries(att_setpoint   ${catkin_LIBRARIES} )
 target_link_libraries(inclined_landing   ${catkin_LIBRARIES} )
 
