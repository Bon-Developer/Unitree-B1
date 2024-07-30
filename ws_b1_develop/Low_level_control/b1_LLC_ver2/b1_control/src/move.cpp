#include <iostream>
#include "b1_base/base.hpp"

using namespace qre;

int main(int argc, char *argv[]) {
    // * =============================== 기본설정 ========================================================================
    ros::init(argc, arg_nh("~");
    Base* base_driver;v, "move");
    ros::NodeHandle nh, p_nh("~");
    UT::LoopFunc* control_loop;
    UT::InitEnvironment();
    if (argc >= 2) {
        if (strcmp(argv[1], "high_level") == 0) {
            std::cout << "Working mode is set to: " << argv[1] << std::endl;
            UT::HighCmd cmd;
            UT::HighState state;
            base_driver  = new Base(cmd, state, &nh, &p_nh);
            control_loop = new UT::LoopFunc("control_loop", base_driver->dt, boost::bind(&Base::HighLevelControl, base_driver));
        }
        else if (strcmp(argv[1], "low_level") == 0) {
            std::cout << "Working mode is set to: " << argv[1] << std::endl;                
            UT::LowCmd cmd;
            UT::LowState state;
            base_driver  = new Base(cmd, state, &nh, &p_nh);
            control_loop = new UT::LoopFunc("control_loop", base_driver->dt, boost::bind(&Base::LowLevelControl, base_driver));
        }
        else {
            std::cout << "Invalid working mode" << std::endl;
            std::cout << "Supported modes_ are: high_level and low_level" << std::endl;
            exit(-1);
        }
    }
    else {
        std::cout << "Working mode not specified" << std::endl;
        std::cout << "Supported modes_ are: high_level and low_level" << std::endl;
        exit(-1);
    }    
    UT::LoopFunc comm_bus_publisher("udp_send",  base_driver->dt, 3, boost::bind(&Base::packetSend,    base_driver));
    UT::LoopFunc comm_bus_subscriber("udp_recv", base_driver->dt, 3, boost::bind(&Base::packetReceive, base_driver));
    comm_bus_publisher.start();
    comm_bus_subscriber.start();
    control_loop->start();
    ros::Rate loop_rate(50); // 50hz
    // * ======================================================================================================================

    // TODO -> STEP 1 : RRT* node로부터 정보를 받을 subscriber를 정의한다.
    // TODO -> STEP 2 : subscriber의 callback함수 정의 --> base.cpp에 cmdVelCallback쓰면 될 거 같다.(그 대신 geometry_msgs/Twist 써야함)
    // ! : topic을 보낼 때, linear.y, linear.z, angular.x, angular.y를 0이여야해서 error 체크 해야됨.
    ros::Subscriber velocity_subscriber = p_nh->subscribe("move", 1, base_driver->cmdVelCallback);

    // TODO -> STEP 3 : Emergency 객체 완성 후, key monitoring 할 수 있도록 설정.(champ_teleop의 getKey함수를 사용하면 될 거 같다.)

    // TODO -> STEP 4 : 

    while (ros::ok()) {
        base_driver->publishStateMessages(); 
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
