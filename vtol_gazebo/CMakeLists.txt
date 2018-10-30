cmake_minimum_required(VERSION 2.8.3)
project(vtol_gazebo)

add_definitions(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    gazebo_plugins
    gazebo_ros
    roscpp
    urdf
    xacro
    tf
)

find_package(Eigen3 REQUIRED)
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)
catkin_package(
  INCLUDE_DIRS
    include
    ${GAZEBO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    gazebo_plugins
    urdf
    xacro
    DEPENDS
    roscpp
    gazebo_ros
    Boost
    tf
)



add_library(rotor_gazebo
  src/RotorGazeboPlugin.cpp
)
add_dependencies(rotor_gazebo
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(rotor_gazebo
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

add_library(double_rotor_gazebo
  src/DoubleRotorGazeboPlugin.cpp
)
add_dependencies(double_rotor_gazebo
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(double_rotor_gazebo
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
add_library(vtol_base_gazebo
  src/VtolBaseGazeboPlugin.cpp
)
add_dependencies(vtol_base_gazebo
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(vtol_base_gazebo
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
