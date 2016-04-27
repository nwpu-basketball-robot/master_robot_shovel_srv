cmake_minimum_required(VERSION 2.8.3)
project(basketball_shovel_srv)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS

  basketball_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES basketball_shovel_srv
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# catkin_add_nosetests(test)
add_executable(robot_shovel_srv src/basketball_shovel_service.cpp)
target_link_libraries(robot_shovel_srv ${catkin_LIBRARIES})
 add_dependencies(robot_shovel_srv basketball_msgs_gencpp)
