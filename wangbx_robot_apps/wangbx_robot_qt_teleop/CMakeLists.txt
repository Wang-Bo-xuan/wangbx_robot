cmake_minimum_required(VERSION 2.8.11)
project(wangbx_robot_qt_teleop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
)

find_package(Qt5Widgets REQUIRED)
qt5_wrap_cpp(MOC src/qt_ui.h)
#qt5_wrap_ui(UIC src/qt_ros_test.ui)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

include_directories(
  INCLUDE_DIRS include
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
   INCLUDE_DIRS
   CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs
   DEPENDS
)

add_executable(qt_teleop src/main.cpp src/qt_ui.cpp)
add_dependencies(qt_teleop ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(qt_teleop ${catkin_LIBRARIES} Qt5::Widgets)

