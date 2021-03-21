cmake_minimum_required(VERSION 2.8.3)
project(turtlesim_controller)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  turtlesim
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES turtlesim_controller_lib
  CATKIN_DEPENDS geometry_msgs roscpp turtlesim
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(turtlesim_controller_lib src/turtlesim_controller.cpp)
add_dependencies(turtlesim_controller_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME} src/turtlesim_controller.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  turtlesim_controller_lib
)
