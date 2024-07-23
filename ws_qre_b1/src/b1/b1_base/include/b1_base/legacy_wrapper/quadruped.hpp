#ifndef _QUADRUPED_H
#define _QUADRUPED_H

#include <ros/ros.h>
#include "robot.hpp"
#include <Eigen/Core> //c++ 라이브러리
#include <Eigen/Geometry> //c++ 라이브러리
#include <sensor_msgs/JointStat.h>

class Quadruped : public qre::Robot {
  Eigen::Matrix4d b1_config_matrix[4];
  const double trunk_length = 0.72/2;
  
