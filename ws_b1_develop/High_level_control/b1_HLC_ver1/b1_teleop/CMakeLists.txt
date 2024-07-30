cmake_minimum_required(VERSION 2.8.3)
project(b1_teleop)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
   b1_teleop.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

