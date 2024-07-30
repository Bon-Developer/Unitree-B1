cmake_minimum_required(VERSION 3.10)
project(b1_base VERSION 0.5.0)
add_compile_options(-std=c++17)
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(SYSTEM_ARCHITECTURE amd64)
endif()
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
  set(SYSTEM_ARCHITECTURE arm64)
endif()
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  roslib
  b1_legged_msgs 
  kdl_parser 
  urdf
  cmake_modules REQUIRED  
)
find_package(Eigen3 REQUIRED)
catkin_package(INCLUDE_DIRS include)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

link_directories(lib/${SYSTEM_ARCHITECTURE})
set(LIB_UNITREE -pthread libunitree_legged_sdk.so)
set(CMAKE_CXX_FLAGS "-O3 -fPIC")
add_executable(base_node src/legacy_wrapper/robot.cpp
                         src/legacy_wrapper/quadruped.cpp
                         src/base_position.cpp
                         src/base_node.cpp)
target_link_libraries(base_node ${LIB_UNITREE} ${catkin_LIBRARIES})
