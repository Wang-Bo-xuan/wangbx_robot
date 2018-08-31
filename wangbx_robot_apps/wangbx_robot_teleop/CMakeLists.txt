cmake_minimum_required(VERSION 2.8.3)
project(wangbx_robot_teleop)

find_package(catkin REQUIRED COMPONENTS
            roscpp
            geometry_msgs
            joy)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp geometry_msgs joy
  DEPENDS
)

add_executable(wangbx_robot_keyboard_teleop src/keyboard_teleop.cpp)
target_link_libraries(wangbx_robot_keyboard_teleop ${catkin_LIBRARIES})
