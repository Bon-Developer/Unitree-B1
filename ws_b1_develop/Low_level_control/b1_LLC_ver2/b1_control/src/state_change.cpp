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

    ros::Subscriber mode_subscriber_ = p_nh->subscribe("set_mode",1, base_driver->modeCallback);
    ros::Subscriber body_height_subscriber_ = p_nh->subscribe("set_body_height",1, base_driver->bodyHeightCallback);
    ros::Subscriber foot_raise_height_subscriber_ = p_nh->subscribe("set_foot_raise_height",1, base_driver->footRaiseHeightCallback);

    while (ros::ok()) {
        base_driver->publishStateMessages(); 
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
