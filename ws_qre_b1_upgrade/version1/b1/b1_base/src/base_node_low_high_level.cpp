#include <iostream>
#include "b1_base/base_low_high_level.hpp"

using namespace qre;

// double jointLinearInterpolation(double initPos, double targetPos, double rate);

int main(int argc, char *argv[]) {

    // * ========================================================== 로봇 기본세팅 ====================================================================

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
    UT::LoopFunc comm_bus_publisher("udp_send",  base_driver->dt, 3, boost::bind(&Base::packetSend,    base_driver)); // dt = 0.002 -> 500hz
    UT::LoopFunc comm_bus_subscriber("udp_recv", base_driver->dt, 3, boost::bind(&Base::packetReceive, base_driver));
    comm_bus_publisher.start();
    comm_bus_subscriber.start();
    control_loop->start();
    ros::Rate loop_rate(50);

    // * =========================== SangTae's code =======================================================
    
    // low level control_loop에 &Base::HighLevelControl 함수를 전달했는데 이는 로봇의 정보를 받고, 전달해주는 것만 하는 함수다.
    // 따라서, robot_low_cmd를 전달하는데 이 안에 변수를 바꾸면 제어가 될 듯하다.
    // UT::Lowcmd robot_low_cmd에 motorcmd가 있는데 motorcmd의 q,dq를 바꾸면 위치와 속도제어가 가능할 것 같다.

    std::cout << "=========================================" << std::endl;
    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
}
    // // * 프로그램 실행에 필요한 변수들
    // int motiontime = 0;
    // int elasped_time = 0;
    // float qInit[3] = {0};
    // float qDes[3] = {0};
    // float sin_mid_q[3] = {0.0, 1.2, -2.0};
    // float Kp[3] = {0};
    // float Kd[3] = {0};
    // double time_consume = 0;
    // int rate_count = 0;
    // int sin_count = 0;
    // float dt = 0.002; // 0.001~0.01

    // * keyboard teleop으로부터 정보를 받아오는 노드의 핸들

//     while (ros::ok()) {

//         // robot_control에 관한 control loop만들기
//         motiontime++;
//         if(motiontime == 50){ // 1초에 한번씩 출력하도록
//             elasped_time = motiontime / 50.0; // 50은 ros::Rate 객체의 주기 // ! int나누기float을 하면 어떻게 되더라?
//             printf("FR_0 : %d  %f  %f\n", elasped_time, base_driver->base_low_state.motorState[UT::FR_0].q, base_driver->base_low_state.motorState[UT::FR_0].dq);
//             printf("FR_1 : %d  %f  %f\n", elasped_time, base_driver->base_low_state.motorState[UT::FR_1].q, base_driver->base_low_state.motorState[UT::FR_1].dq);
//             printf("FR_2 : %d  %f  %f\n", elasped_time, base_driver->base_low_state.motorState[UT::FR_2].q, base_driver->base_low_state.motorState[UT::FR_2].dq);
//             printf("\n\n");
//         }

//         //gravity compensation
//         base_driver->robot_low_cmd.motorCmd[UT::FR_0].tau = -5.0f;

//         // if( motiontime >= 100){
//         if (motiontime >= 0)
//         {
//             // first, get record initial position
//             if (motiontime >= 0 && motiontime < 10)
//             {
//                 qInit[0] = base_driver->base_low_state.motorState[UT::FR_0].q;
//                 qInit[1] = base_driver->base_low_state.motorState[UT::FR_1].q;
//                 qInit[2] = base_driver->base_low_state.motorState[UT::FR_2].q;
//             }
//             // second, move to the origin point of a sine movement with Kp Kd
//             if (motiontime >= 10 && motiontime < 400)
//             {
//                 rate_count++;
//                 double rate = rate_count / 200.0; // needs count to 200
//                 // Kp[0] = 5.0; Kp[1] = 5.0; Kp[2] = 5.0;
//                 // Kd[0] = 1.0; Kd[1] = 1.0; Kd[2] = 1.0;
//                 Kp[0] = 20.0;
//                 Kp[1] = 20.0;
//                 Kp[2] = 20.0;
//                 Kd[0] = 2.0;
//                 Kd[1] = 2.0;
//                 Kd[2] = 2.0;

//                 qDes[0] = jointLinearInterpolation(qInit[0], sin_mid_q[0], rate);
//                 qDes[1] = jointLinearInterpolation(qInit[1], sin_mid_q[1], rate);
//                 qDes[2] = jointLinearInterpolation(qInit[2], sin_mid_q[2], rate);
//             }
//             double sin_joint1, sin_joint2;
//             // last, do sine wave
//             float freq_Hz = 1;
//             // float freq_Hz = 5;
//             float freq_rad = freq_Hz * 2 * M_PI;
//             float t = dt * sin_count;
//             if (motiontime >= 400)
//             {
//                 sin_count++;
//                 // sin_joint1 = 0.6 * sin(3*M_PI*sin_count/1000.0);
//                 // sin_joint2 = -0.9 * sin(3*M_PI*sin_count/1000.0);
//                 sin_joint1 = 0.6 * sin(t * freq_rad);
//                 sin_joint2 = -0.9 * sin(t * freq_rad);
//                 qDes[0] = sin_mid_q[0];
//                 qDes[1] = sin_mid_q[1] + sin_joint1;
//                 qDes[2] = sin_mid_q[2] + sin_joint2;
//             }

//             base_driver->robot_low_cmd.motorCmd[UT::FR_0].q = qDes[0];
//             base_driver->robot_low_cmd.motorCmd[UT::FR_0].dq = 0;
//             base_driver->robot_low_cmd.motorCmd[UT::FR_0].Kp = Kp[0];
//             base_driver->robot_low_cmd.motorCmd[UT::FR_0].Kd = Kd[0];
//             base_driver->robot_low_cmd.motorCmd[UT::FR_0].tau = -4.0f;

//             base_driver->robot_low_cmd.motorCmd[UT::FR_1].q = qDes[1];
//             base_driver->robot_low_cmd.motorCmd[UT::FR_1].dq = 0;
//             base_driver->robot_low_cmd.motorCmd[UT::FR_1].Kp = Kp[1];
//             base_driver->robot_low_cmd.motorCmd[UT::FR_1].Kd = Kd[1];
//             base_driver->robot_low_cmd.motorCmd[UT::FR_1].tau = 0.0f;

//             base_driver->robot_low_cmd.motorCmd[UT::FR_2].q = qDes[2];
//             base_driver->robot_low_cmd.motorCmd[UT::FR_2].dq = 0;
//             base_driver->robot_low_cmd.motorCmd[UT::FR_2].Kp = Kp[2];
//             base_driver->robot_low_cmd.motorCmd[UT::FR_2].Kd = Kd[2];
//             base_driver->robot_low_cmd.motorCmd[UT::FR_2].tau = 0.0f;
//         }

//         base_driver->publishStateMessages();
//         ros::spinOnce();
//         loop_rate.sleep();
//     }
//     return 0;
// }


// double jointLinearInterpolation(double initPos, double targetPos, double rate)
// {
//     double p;
//     rate = std::min(std::max(rate, 0.0), 1.0);
//     p = initPos * (1 - rate) + targetPos * rate;
//     return p;
// }