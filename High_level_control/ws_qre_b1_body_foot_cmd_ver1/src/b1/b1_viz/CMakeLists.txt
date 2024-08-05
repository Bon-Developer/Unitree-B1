cmake_minimum_required(VERSION 3.0.2)
project(b1_viz)


find_package(catkin REQUIRED COMPONENTS
  rviz
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES b1_viz
#  CATKIN_DEPENDS rviz
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)