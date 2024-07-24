#ifndef _QUADRUPED_H
#define _QUADRUPED_H

#include <ros/ros.h>
#include "robot.hpp"
#include <Eigen/Core> //c++ 라이브러리
#include <Eigen/Geometry> //c++ 라이브러리
#include <sensor_msgs/JointStat.h>

class Quadruped : public qre::Robot {
  Eigen::Matrix4d b1_config_matrix[4]; //로봇의 각 발 4개에 대한 매트릭스 배열 저장
  const double trunk_length = 0.72/2; //body 세로 길이
  const double trunk_width = 0.2335/2; //body 가로 길이
  const double l1 = 0.;
  const double l2 = 0.12; //hip
  const double l3 = 0.39; //thigh
  const double l4 = 0.39; //calf

public:
  Quadruped(); //생성자
  ~Quadruped(); //소멸자
  Eigen::Matrix<double, 4, 1>* footTransformsFromPositions();
  Eigen::Matrix<double, 4, 3>  jointAngleFromFootPositions(Eigen::Matrix<double, 4, 1> *foot_positions);
  sensor_msgs::JointState getJointStates();
  sensor_msgs::JointState extractJointAngles();
  Eigen::Matrix3d rotx(double alpha);
  Eigen::Matrix3d roty(double beta);
  Eigen::Matrix3d rotz(double gamma);
  Eigen::Matrix3d rotxyz(double alpha, double beta, double gamma);
};

#endif
  
  
  
