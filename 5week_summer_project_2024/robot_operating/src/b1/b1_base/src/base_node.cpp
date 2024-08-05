#include <iostream>
#include "b1_base/base.hpp"

using namespace qre;

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "base_node");
    ros::NodeHandle nh, p_nh("~");
    Base* base_driver;
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
    ros::Rate loop_rate(50);

    bool first_change = false;
    bool second_change = false;

    while (ros::ok()) 
    {
        //Robot::extractOdometryMessage()함수로부터 odom에 대한 정보를 알 수 있다.
        auto [odom, odom_transform] = base_driver->extractOdometryMessage();
        if(odom.pose.pose.position.x > 3.0 && first_change == false)
        {
            if(odom.pose.pose.position.x < 5.0)
            {
                base_driver->robot_high_cmd.bodyHeight = - 0.1;
                base_driver->robot_high_cmd.footRaiseHeight = 0.1;
                base_driver->robot_high_cmd.speedLevel = 0;
                printf("gait type changing: obstacle mode\n");
            }
            else
            {
                first_change = true;
            }

        }

        else if(odom.pose.pose.position.x > 5.0 && second_change == false)
        {
            base_driver->robot_high_cmd.bodyHeight = 0.0;
            base_driver->robot_high_cmd.footRaiseHeight = 0.0;
            base_driver->robot_high_cmd.speedLevel = 1;
            printf("gait type changing: walking mode\n");
            second_change = true;
        }

        base_driver->publishStateMessages();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
