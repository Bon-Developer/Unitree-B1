/**
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
**/

/**
code analysis by bon, July 8, 2024
**/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
  public:
    Custom(uint8_t level);
      safe(LeggedType::B1),
      udp(level, 8090, "192.168.123.10", 8007){
      udp.InitCmdData(cmd);
      }
    void UDPRecv();
    void UDPSend();
    void RobotControl();

    Safety safe;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};
    int Tpi = 0;
    int motiontime = 0;
    float dt = 0.002; //0.001 ~ 0.01
};

void Custom::UDPRecv()
{
  udp.Recv();
}

void Custom::UDPSend()
{
  udp.Send();
}

void Custom::RobotControl()
{
  motiontime++;
  udp.GetRecv(state);

  //gravity compensation
  cmd.motorCmd[FR_0].tau = -5.0f;
  cmd.motorCmd[FL_0].tau = +0.65f;
  cmd.motorCmd[RR_0].tau = -0.65f;
  cmd.motorCmd[RL_0].tau = +0.65f;

  if(motiontime >= 500)
    {
      float speed1 = 2 * sin(3 * M_PI * Tpi / 2000.0);
      float speed2 = 2 * sin(3 * M_PI * Tpi / 2000.0 + (1 / 4) * M_PI);
      float speed3 = 2 * sin(3 * M_PI * Tpi / 2000.0 + (2 / 4) * M_PI);
      float speed3 = 2 * sin(3 * M_PI * Tpi / 2000.0 + (3 / 4) * M_PI);

      cmd.motorCmd[FR_1].q = PosStopF;
      cmd.motorCmd[FR_1].dq = speed1;
      cmd.motorCmd[FR_1].Kp = 0;
      cmd.motorCmd[FR_1].Kd = 4;
      cmd.motorCmd[FR_1].tau = 0.0f;

      cmd.motorCmd[FL_1].q = PosStopF;
      cmd.motorCmd[FL_1].dq = speed2;
      cmd.motorCmd[FL_1].Kp = 0;
      cmd.motorCmd[FL_1].Kd = 4;
      cmd.motorCmd[FL_1].tau = 0.0f;

      cmd.motorCmd[RR_1].q = PosStopF;
      cmd.motorCmd[RR_1].dq = speed3;
      cmd.motorCmd[RR_1].Kp = 0;
      cmd.motorCmd[RR_1].Kd = 4;
      cmd.motorCmd[RR_1].tau = 0.0f;

      cmd.motorCmd[RL_1].q = PosStopF;
      cmd.motorCmd[RL_1].dq = speed4;
      cmd.motorCmd[RL_1].Kp = 0;
      cmd.motorCmd[RL_1].Kd = 4;
      cmd.motorCmd[RL_1].tau = 0.0f;

      Tpi++;
    }

  udp.SetSend(cmd);
}

int main(void)
{
  std::cout << "Communication level is set to Low-level." << std::endl
            << "WARNING: Make sure the robot is hung up." << std::endl
            << "Press Enter to continue..." << std::endl;
  std::cin.ignore();

  Custom custom(LOWLEVEL);
  InitEnvironment(); 
  LoopFunc loop_control("control_loop", custom.dt, boost::bind(&Custom::RobotControl, &custom));
  LoopFunc loop_udpSend("udp_send", custom.dt, 3, boost::bind(&Custom::UDPSend, &custom));
  LoopFunc loop_udpRecv("udp_recv", custom.dt, 3, boost::bind(&Custom::UDPRecv. &custom));

  loop_udpSend.start();
  loop_udpRecv.start();
  loop_control.start();

  while(1)
  {
    sleep(10);
  };

  return 0;
}
      
