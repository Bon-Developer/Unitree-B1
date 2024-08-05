cmake_minimum_required(VERSION 3.0.2)
project(b1_bringup)
find_package(catkin REQUIRED COMPONENTS
  b1_base
  b1_control
  b1_description
  robot_upstart
)