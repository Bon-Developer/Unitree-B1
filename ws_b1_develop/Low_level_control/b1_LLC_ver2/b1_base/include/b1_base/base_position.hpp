#ifndef _BASE_H
#define _BASE_H 

#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "legacy_wrapper/quadruped.hpp"
#include "b1_legged_msgs/SetMode.h"
#include "b1_legged_msgs/SetControl.h"
#include "b1_legged_msgs/JointCmd.h"
#include "b1_legged_msgs/SetBodyHeight.h"
#include "b1_legged_msgs/SetFootRaiseHeight.h"


namespace qre{
    void heartbeatMonitor();
    class Base : public Quadruped {
        ros::NodeHandle *nh_, *p_nh_;

        ros::Subscriber cmd_subscriber_;
        ros::Subscriber mode_subscriber_;
        ros::Subscriber body_height_subscriber_;
        ros::Subscriber foot_raise_height_subscriber_; 
       
        ros::Publisher joint_state_publisher_;
        ros::Publisher state_publisher_;
        ros::Publisher imu_publisher_;
        ros::Publisher battery_state_publisher_;
        ros::Publisher motor_state_publisher_;
        ros::Publisher odom_publisher_;  
       
        ros::ServiceServer set_control_;  

        UT::Safety robot_safety_;
        UT::UDP* comm_bus_;

        uint16_t level_;
        std::string modes_[4] = {"Position", "Velocity", "Torque", "Full"};
        std::string low_level_control_type_ = "Position";
        char udp_ip_[16];        

    public:
        Base(UT::HighCmd cmd, UT::HighState state, ros::NodeHandle *nh, ros::NodeHandle *p_nh);
        Base(UT::LowCmd  cmd, UT::LowState  state, ros::NodeHandle *nh, ros::NodeHandle *p_nh);
        void HighLevelControl();
        void LowLevelControl();
        void packetReceive();
        void packetSend();
        void cmdVelCallback(geometry_msgs::Twist msg);
        void jointCommandCallback(b1_legged_msgs::JointCmd msg);
        void publishStateMessages();
        bool setControlCallback(b1_legged_msgs::SetControl::Request &req, b1_legged_msgs::SetControl::Response &res);

        // * Custom function
        void modeCallback(const b1_legged_msgs::SetMode::ConstPtr& msg);
        void bodyHeightCallback(const b1_legged_msgs::SetBodyHeight::ConstPtr& msg);
        void footRaiseHeightCallback(b1_legged_msgs::SetFootRaiseHeight msg);

        double jointLinearInterpolation(double initPos, double targetPos, double rate);
        

        // * LowLevelControl에 사용할 변수들 ==================================
        int motiontime = 0;
        int elasped_time = 0;
        float qInit[3] = {0};
        float qDes[3] = {0};
        float sin_mid_q[3] = {0.0, 1.2, -2.0};
        float Kp[3] = {0};
        float Kd[3] = {0};
        double time_consume = 0;
        int rate_count = 0;
        int sin_count = 0;
        float dt = 0.002;
        // * ===============================================================
    };
}

#endif 
