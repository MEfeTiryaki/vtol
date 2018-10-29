cmake_minimum_required(VERSION 2.8.3)
project(vtol_controller)


find_package(catkin REQUIRED COMPONENTS
  controller_manager
)

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
