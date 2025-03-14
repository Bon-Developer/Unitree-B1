#ifndef _QUADRUPED_H
#define _QUADRUPED_H 

#include <ros/ros.h>
#include "robot.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry> 
#include <sensor_msgs/JointState.h>


class Quadruped : public qre::Robot {
	Eigen::Matrix4d b1_config_matrix[4];
	const double trunk_length = 0.72/2;
	const double trunk_width  = 0.2335/2;
	const double l1 = 0.;
	const double l2 = 0.12; // hip
	const double l3 = 0.39; // thigh
	const double l4 = 0.39; // calf   

public:
	Quadruped();
	~Quadruped();
	Eigen::Matrix<double, 4, 1>* footTransformsFromPositions();
  	Eigen::Matrix<double, 4, 3>  jointAnglesFromFootPositions(Eigen::Matrix<double, 4, 1> *foot_positions);
	sensor_msgs::JointState getJointStates();
	sensor_msgs::JointState extractJointAngles();
	Eigen::Matrix3d rotx(double alpha);
	Eigen::Matrix3d roty(double beta);
	Eigen::Matrix3d rotz(double gamma);
	Eigen::Matrix3d rotxyz(double alpha, double beta, double gamma);
};

#endif 
