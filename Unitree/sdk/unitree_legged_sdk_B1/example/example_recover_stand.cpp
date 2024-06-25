/**
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
**/

/**
code analysis by bon, June 24, 2024
**/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
  public:
    Custom(uint8_t level) : safe(LeggedType::B1),
                            udp(level, 8090, "192.168.123.220", 8082)
  {
    udp.InitCmdData(cmd);
    //udp.print = true;
  }
  void UDPRecv();
  void UDPSend();
  void RobotControl();

  Safety safe; 
  UDP udp;
  HighCmd cmd = {0};
  HighState state = {0};
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
  motiontime += 2;
  udp.GetRecv(state);
  printf("%d %f\n", motiontime, state.imu.rpy[2]);

  cmd.mode = 0; //0: idle, default stand, 1: forced stand, 2: walk continuously, high cmd에서 mode 선택
  cmd.gaitType = 0; //gait type 지정, 0: idle
  cmd.speedLevel = 0; //0: default low speed
  cmd.footRaiseHeight = 0; //기본값 0.08인데 0으로 주는거면 발을 아예 들지 않는 것인지 확인 필요
  cmd.bodyHeight = 0; //기본값 0.28인데 얘도 확인 필요
  cmd.euler[0] = 0;
  cmd.euler[1] = 0;
  cmd.euler[2] = 0;
  cmd.velocity[0] = 0.0f;
  cmd.velocity[1] = 0.0f;
  cmd.yawSpeed = 0.0f;
  cmd.reserve = 0;

  if(motiontime >= 0 && motiontime < 3000)
  {
    cmd.mode = 9; //Make sure the robot lie on the ground upside down, otherwise it wouldn't work, 9: recover stand
  }
  else
  {
    cmd.mode = 0; //0: idle(default stand)
  }

  udp.SetSend(cmd);
}

int main(void)
{
  std::cout << "Communication level is set to HIGH-level." << std::endl
            << "WARNING: Make sure the robot lie on the ground upside down, otherwise it wouldn't work." << std::endl
            << "Press Enter to continue..." << std::endl;
  std::cin.ignore();

  Custom custom(HIGHLEVEL);
  InitEnvironment();
  LoopFunc loop_control("control_loop", custom.dt, boost::bind(&Custom::RobotControl, &custom));
  LoopFunc loop_udpSend("udp_send", custom.dt, 3, boost::bind(&Custom::UDPSend, &custom));
  LoopFunc loop_udpRecv("udp_recv", custom.dt, 3, boost::bind(&Custom::UDPRecv, &custom));

  loop_udpSend.start();
  loop_udpRecv.start();
  loop_control.start();

  while(1)
  {
    sleep(10);
  };

  return 0;
}


















