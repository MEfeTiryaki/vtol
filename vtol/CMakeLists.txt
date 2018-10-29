cmake_minimum_required(VERSION 2.8.3)
project(vtol)


find_package(catkin REQUIRED COMPONENTS
  vtol_description
)

catkin_package(
  CATKIN_DEPENDS
  vtol_description
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
