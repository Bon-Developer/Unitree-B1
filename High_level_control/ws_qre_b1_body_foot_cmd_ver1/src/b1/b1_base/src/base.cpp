#include "b1_base/base.hpp"

using namespace qre;
Base::Base(UT::HighCmd cmd, UT::HighState state, ros::NodeHandle *nh, ros::NodeHandle *p_nh): robot_safety_(UT::LeggedType::B1) {
    nh_ = nh;
    p_nh_ = p_nh;
    level_ = UT::HIGHLEVEL;
    robot_high_cmd.mode            = 2;
    robot_high_cmd.gaitType        = 2;
    robot_high_cmd.velocity        = {0.0};
    robot_high_cmd.position        = {0.0};
    robot_high_cmd.yawSpeed        = 0.0;
    robot_high_cmd.euler           = {0.0};
    robot_high_cmd.bodyHeight      = 0.0;
    robot_high_cmd.footRaiseHeight =  0.0;
    odom_publisher_  = p_nh_->advertise<nav_msgs::Odometry>("odom", 1);
    cmd_subscriber_  = p_nh_->subscribe("cmd_vel", 1, &Base::cmdVelCallback, this);
    state_publisher_ = p_nh_->advertise<b1_legged_msgs::HighState>("state", 1);
    imu_publisher_   = p_nh_->advertise<sensor_msgs::Imu>("imu/data", 1);
    motor_state_publisher_   = nh_->advertise<b1_legged_msgs::MotorStateArray>("motor_states", 1);
    battery_state_publisher_ = nh_->advertise<sensor_msgs::BatteryState>("battery_state", 1);
    joint_state_publisher_   = nh_->advertise<sensor_msgs::JointState>("joint_states", 10);
    // set_mode_ = p_nh_->advertiseService("set_mode_", &Base::setModeCallback, this);

    mode_subscriber_ = p_nh_->subscribe("set_mode",1, &Base::modeCallback, this);
    body_height_subscriber_ = p_nh_->subscribe("set_body_height",1, &Base::bodyHeightCallback, this);
    foot_raise_height_subscriber_ = p_nh_->subscribe("set_foot_raise_height",1, &Base::footRaiseHeightCallback, this);
    
    std::string ip_string;
    p_nh_->param<std::string>("target_ip", ip_string, "192.168.123.220");
    strcpy(udp_ip_, ip_string.c_str());
    int target_port, local_port;
    p_nh_->param<int>("target_port", target_port, 8082);
    p_nh_->param<int>("local_port",  local_port,  8090);
    std::cout << "Initializing UDP with ip: " << ip_string << ", local port: " << local_port << ", target port: " << target_port << std::endl; 
    comm_bus_ = new UT::UDP(level_, local_port, const_cast<const char*>(udp_ip_), target_port);
    comm_bus_->InitCmdData(robot_high_cmd);
}

Base::Base(UT::LowCmd  cmd, UT::LowState  state, ros::NodeHandle *nh, ros::NodeHandle *p_nh): robot_safety_(UT::LeggedType::B1) {
    nh_   = nh;
    p_nh_ = p_nh;
    level_ = UT::LOWLEVEL;
    cmd_subscriber_  = p_nh_->subscribe("joint_cmd", 1, &Base::jointCommandCallback, this);
    state_publisher_ = p_nh_->advertise<b1_legged_msgs::LowState>("state", 1);
    imu_publisher_   = p_nh_->advertise<sensor_msgs::Imu>("imu/data", 1);
    joint_state_publisher_   = nh_->advertise<sensor_msgs::JointState>("joint_states", 10);
    battery_state_publisher_ = nh_->advertise<sensor_msgs::BatteryState>("battery_state", 1);
    motor_state_publisher_   = p_nh_->advertise<b1_legged_msgs::MotorStateArray>("motor_states", 1);
    set_control_ = p_nh_->advertiseService("set_control_", &Base::setControlCallback, this);
    for(int i = 0; i<12; i++) {
        robot_low_cmd.motorCmd[i].mode = 0x0A;  
        robot_low_cmd.motorCmd[i].q    = UT::PosStopF;        
        robot_low_cmd.motorCmd[i].Kp   = 0;
        robot_low_cmd.motorCmd[i].dq   = UT::VelStopF;
        robot_low_cmd.motorCmd[i].Kd   = 0;
        robot_low_cmd.motorCmd[i].tau  = 0;
    }
    ros::param::set("~/control_type", "Position");
    std::string ip_string;    
    p_nh_->param<std::string>("target_ip", ip_string, "192.168.123.10");
    strcpy(udp_ip_, ip_string.c_str());
    int target_port, local_port;
    p_nh_->param<int>("target_port", target_port, 8007);
    p_nh_->param<int>("local_port",  local_port,  8090);
    std::cout << "Initializing UDP with ip: " << ip_string << ", local port: " << local_port << ", target port: " << target_port << std::endl; 
    comm_bus_ = new UT::UDP(level_, local_port, const_cast<const char*>(udp_ip_), target_port);
    comm_bus_->InitCmdData(robot_low_cmd);
}

void Base::packetReceive() {
    comm_bus_->Recv();
}

void Base::packetSend() {  
    comm_bus_->Send();
}

void Base::HighLevelControl() {
    comm_bus_->GetRecv(base_high_state);
    comm_bus_->SetSend(robot_high_cmd);
}

void Base::LowLevelControl() {
    comm_bus_->GetRecv(base_low_state);
    comm_bus_->SetSend(robot_low_cmd);
}

void Base::publishStateMessages() {
    if (level_ == UT::HIGHLEVEL) {
        auto state_msg = extractHighStateMessage();
        auto joint_state_msg = getJointStates();
        auto [odom, odom_transform] = extractOdometryMessage();
        // odom_broadcaster_.sendTransform(odom_transform);
        odom_publisher_.publish(odom);
        joint_state_publisher_.publish(joint_state_msg);
        state_publisher_.publish(state_msg);
    }
    else if (level_ == UT::LOWLEVEL) {
        auto state_msg   = extractLowStateMessage();
        auto joint_angle = extractJointAngles();
        state_publisher_.publish(state_msg);
        joint_state_publisher_.publish(joint_angle);
    }
    auto battery_state_msg = extractBatteryStateMessage();
    auto motor_state_msg   = extractMotorStateMessage();
    auto imu_msg = extractImuMessage();
    battery_state_publisher_.publish(battery_state_msg);
    motor_state_publisher_.publish(motor_state_msg);
    imu_publisher_.publish(imu_msg);
}

// bool Base::setModeCallback(b1_legged_msgs::SetMode::Request &req, b1_legged_msgs::SetMode::Response &res) {
//     robot_high_cmd.mode = req.mode;
//     robot_high_cmd.gaitType = req.gait_type;
//     robot_high_cmd.speedLevel = req.speed_level;
//     res.success = true;
//     return true;
// }

bool Base::setControlCallback(b1_legged_msgs::SetControl::Request &req, b1_legged_msgs::SetControl::Response &res) {
    if(std::any_of(std::begin(modes_), std::end(modes_), [=](std::string mode) {return mode == req.control;})) {    
        low_level_control_type_ = req.control;
        ros::param::set("~/control_type", low_level_control_type_);
        res.success = true;
        res.message = "Low level_ control changed to: " + low_level_control_type_;
        return true;
        // }
    }
    else {
        res.success = false;
        res.message = "Select low level_ control mode to Position, Velocity, Torque or Full";
        return false; 
    }   
}

void Base::cmdVelCallback(geometry_msgs::Twist msg) {
    robot_high_cmd.mode=2;
    robot_high_cmd.gaitType=2;
    robot_high_cmd.velocity[0] = 0.;
    robot_high_cmd.velocity[1] = 0.;
    robot_high_cmd.yawSpeed = 0.;        
    if (msg.linear.x && msg.linear.y) {
        robot_high_cmd.velocity[0] = msg.linear.x;
        robot_high_cmd.velocity[1] = msg.linear.y;
    }
    else if (msg.linear.x && msg.angular.z) {
        robot_high_cmd.velocity[0] = msg.linear.x;
        robot_high_cmd.yawSpeed = msg.angular.z;
    }
    else if (msg.linear.x) {
        robot_high_cmd.velocity[0] = msg.linear.x;        
    }
    else if (msg.linear.y) {
         robot_high_cmd.velocity[1] = msg.linear.y;        
    }
    else if (msg.angular.z) {
        robot_high_cmd.yawSpeed = msg.angular.z;
    }
    else {
        robot_high_cmd.mode=0;
    } 
}

void Base::jointCommandCallback(b1_legged_msgs::JointCmd msg) {
        if (low_level_control_type_ != "Torque" && low_level_control_type_ != "Full") {
        robot_low_cmd.motorCmd[UT::FR_0].tau = -0.65f;
        robot_low_cmd.motorCmd[UT::FL_0].tau = +0.65f;
        robot_low_cmd.motorCmd[UT::RR_0].tau = -0.65f;
        robot_low_cmd.motorCmd[UT::RL_0].tau = +0.65f;
    }
    if(low_level_control_type_ == "Position") {
        for (uint16_t i=0; i<12; i++) {
            robot_low_cmd.motorCmd[i].q  = msg.q[i]; 
            robot_low_cmd.motorCmd[i].Kp = msg.Kp[i]; 
            robot_low_cmd.motorCmd[i].Kd = msg.Kd[i]; 
        }
    }
    else if(low_level_control_type_ == "Velocity") {
        for (uint16_t i=0; i<12; i++) {
            robot_low_cmd.motorCmd[i].q  = UT::PosStopF;          
            robot_low_cmd.motorCmd[i].Kp = msg.Kp[i];
            robot_low_cmd.motorCmd[i].Kd = msg.Kd[i];
            robot_low_cmd.motorCmd[i].dq = msg.dq[i];
        }
    }
    else if(low_level_control_type_ == "Torque") {
        for (uint16_t i=0; i<12; i++) {
            robot_low_cmd.motorCmd[i].q   = UT::PosStopF; 
            robot_low_cmd.motorCmd[i].dq  = UT::VelStopF; 
            robot_low_cmd.motorCmd[i].tau = msg.tau[i];
        }
    }   
    else if (low_level_control_type_ == "Full"){
        for (uint16_t i=0; i<12; i++) {
            robot_low_cmd.motorCmd[i].q    = msg.q[i];
            robot_low_cmd.motorCmd[i].dq   = msg.dq[i];
            robot_low_cmd.motorCmd[i].tau  = msg.tau[i];
            robot_low_cmd.motorCmd[i].Kp = msg.Kp[i];
            robot_low_cmd.motorCmd[i].Kd = msg.Kd[i];
        }
    }
  
}

void Base::modeCallback(const b1_legged_msgs::SetMode::ConstPtr& msg)
{
    robot_high_cmd.mode = msg->mode;
    robot_high_cmd.gaitType = msg->gait_type;
    robot_high_cmd.speedLevel = msg->speed_level;
    std::cout << "currently\t mode : " << (int)robot_high_cmd.mode << std::endl;
    std::cout << "currently\t gait type : " << (int)robot_high_cmd.gaitType<< std::endl;
    std::cout << "currently\t speedLevel : " << (int)robot_high_cmd.speedLevel << std::endl;
}

void Base::bodyHeightCallback(const b1_legged_msgs::SetBodyHeight::ConstPtr& msg)
{
    robot_high_cmd.bodyHeight += msg->value;
    std::cout << "currently\t body height : " << robot_high_cmd.bodyHeight << std::endl;
}

void Base::footRaiseHeightCallback(b1_legged_msgs::SetFootRaiseHeight msg)
{
    robot_high_cmd.footRaiseHeight += msg.value;
    std::cout << "currently\t foot raise height : " << robot_high_cmd.footRaiseHeight << std::endl;
}

void qre::heartbeatMonitor() {  
    std::string ip_address;    
    ros::param::param<std::string>("target_ip", ip_address, "192.168.123.161");
    ros::Rate loop_rate(10);
    uint16_t count(0);
    while (ros::ok()) {
        std::string command = std::string("ping -c1 -w1 -s1 ") + ip_address + std::string("  > /dev/null 2>&1");
        int status = system(command.c_str());
        if (status!=0) {
            ROS_WARN("No connection to robot found.");
            ros::shutdown();
            count ++;
            if (count > 10) {
                ROS_ERROR("\nShutting down controller.");
                ros::shutdown();
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout << "\nStopping heartbeat monitor" << std::endl;
}

