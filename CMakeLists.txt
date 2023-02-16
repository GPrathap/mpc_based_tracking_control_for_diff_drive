cmake_minimum_required(VERSION 3.2)
project(nav_husky_robot)

set(CMAKE_BUILD_TYPE "DEBUG")
set(CMAKE_CXX_FLAGS "-std=c++11 -fPIC -fvisibility=hidden")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g --debug-cpp --trace-expand -fvisibility=hidden")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_MATH_DISABLE_FLOAT128 -O1 -Wall -g --debug -fvisibility=hidden")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  actionlib_msgs
  rospy
  move_base_msgs
  message_generation
  nav_msgs
)

add_action_files(
  FILES
  ExecuteControl.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs move_base_msgs geometry_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES nav_husky_robot
    CATKIN_DEPENDS  std_msgs nav_msgs tf rospy actionlib_msgs message_generation geometry_msgs 
                    tf2_ros tf2_geometry_msgs visualization_msgs
    DEPENDS system_lib
    DEPENDS message_runtime                
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
)


add_executable(nav_husky_robot_node src/path_sampling.cpp)
target_link_libraries(nav_husky_robot_node ${catkin_LIBRARIES})

