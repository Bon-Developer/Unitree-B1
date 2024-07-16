// Analysis by bon, July 16, 2024

#ifndef _BASE_H
#define _BASE_H 

#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h> //분석 필요
#include <sensor_msgs/Imu.h> //분석 필요
#include "unitree_legged_sdk/unitree_legged_sdk.h" //분석 필요
#include "legacy_wrapper/quadruped.hpp" //분석 필요
#include "b1_legged_msgs/SetMode.h" //분석 필요
#include "b1_legged_msgs/SetControl.h" //분석 필요
#include "b1_legged_msgs/JointCmd.h" //분석 필요

namespace qre{
  void heartbeatMonitor();
  class Base : public Quadruped {
    ros::NodeHandle *nh_, *p_nh_;

    //서브스크라이버
    ros::Subscriber cmd_subscriber_;

    //퍼블리셔
    ros::Publisher joint_state_publisher_;
    ros::Publisher state_publisher_;
    ros::Publisher imu_publiser_;
    ros::Publisher battery_state_publisher_;
    ros::Publisher motor_state_publisher_;
    // tf::TransformBroadcaster odom_broadcaster_;

    //서비스서버
    ros::ServiceServer set_mode_;
    ros::ServiceServer set_control_;

    UT::Safety robot_safety_;
    UT::UDP* comm_bus_;
    uint16_t level_;
    std::string modes_[4] = {"Position", "Velocity", "Torque", "Full"}; //제어 모드 설정
    std::string low_level_control_type_ = "Position";
    char udp_ip_[16];

  public:
    float dt = 0.002; //0.001 ~ 0.01
    Base(UT::Highcmd cmd, UT::HighState state, ros::NodeHandle *nh, ros::NodeHandle *p_nh);
    Base(UT::LowCmd cmd, UT::LowState state, ros::NodeHandle *nh, ros::NodeHandle *p_nh);
    void HighLevelControl();
    void LowLevelControl();
    void packetReceive();
    void packetSend();
    void cmdVelCallback(geometry_msgs::Twist msg);
    void jointCommandCallback(b1_legged_msgs::JointCmd msg);
    void publishStateMessages();
    bool setModeCallback(b1_legged_msgs::SetMode::Request &req, b1_legged_msgs::SetMode::Response &res);
    bool setControlCallback(b1_legged_msgs::SetControl::Request &req, b1_legged_msgs::SetControl::Response &res);
  };
}

#endif















