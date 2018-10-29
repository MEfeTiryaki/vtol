cmake_minimum_required(VERSION 2.8.3)
project(vtol_sim)

find_package(catkin REQUIRED
  COMPONENTS
    vtol_description
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
    vtol_description
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)
