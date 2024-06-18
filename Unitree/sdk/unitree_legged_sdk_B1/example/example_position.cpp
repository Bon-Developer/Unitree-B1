/**
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
**/

/**
Code anlaysis by bon, June 18, 2024
**/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>

using namespace std;
using namespace UNITREE_LEGGED_SDK;

class Custom
{
  public:
    Custom(uint8_t level) : safe(LeggedType::B1),
                            udp(level, 8090, "192.168.123.10", 8007)
    {
      udp.InitCmdData(cmd);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();

    Safey safe;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};
    float qInit[3] = {0}; //초기 관절 각도 값, 3개 저장
    float qDes[3] = {0}; //목표 관절 각도 값, 3개 저장
    float sin_mid_q[3] = {0.0, 1.2, -2.0}; //사인파 형태의 중간 목표 각도 값, 0.0, 1.2, -2,0은 초기값
    float Kp[3] = {0}; //P 게인, 3개
    float Kd[3] = {0}; //D 게인, 3개
    double time_consume = 0; //연산에 소요되는 시간 저장
    int rate_count = 0; //주기 카운트 변수
    int sin_count = 0; //사인파 주기 카운트 변수
    int motiontime = 0; //동작 시간 카운트 변수
    float dt = 0.002; //0.001~0.01, 제어 주기 시간
};

void Custom::UDPRecv()
{
  udp.Recv();
}

void Custom::UDPSend()
{
  udp.Send();
}

//초기 위치와 목표 위치 사이의 선형 보간을 수행하여 특정 비율에서의 위치 계산
double jointLinearInterpolation(double initPos, double targetPos, double rate) //initPos: 초기 위치(초기 관절 각도), targetPos: 목표 위치(목표 관절 각도), rate: 보간 비율(0.0 ~ 0.1)
{
  double p; //보간된 위치를 저장하는 변수
  rate = std::min(std::max(rate, 0.0), 1.0); //rate가 0.0 ~ 1.0 사이 값이 되도록 제한, 0보다 작으면 0, 1보다 크면 1로 제한
  p = initPos * (1 - rate) + targetPos * rate; //선형 보간 공식을 사용하여 보간된 위치 p 계산, rate = 0.0 -> p = initPos, rate = 1.0 -> p = targetPos, rate = 0.5 -> p = initPos와 targetPos 중간 값 
  return p;
}

void Custom::RobotControl()
{
  motiontime++;
  udp.GetRecv(state);
  //printf("%d %f\n", motiontime, state.motorState[FR_2].q);
  printf("%d %f ^%f\n", motiontiome, state.motorState[FR_1].q, state.motorState[FR_1].dq);

  //graivty compensation
  cmd.motorCmd[FR_0].tau = -5.0f; //첫 번째 관절에 중력 보상 토크 적용
  //cmd.motorCmd[FL_0].tau = +0.65f;
  //cmd.motorCmd[RR_0].tau = -0.65f;
  //cmd.motorCmd[RL_0].tau = 0.65f;

  //if(motiontime >= 100){ 
  if(motiontime >= 0)
  {
    //first, get record initial position
    //if(motiontime >= 100 && motiontime <= 500){
    if(motiontime >= 0 && motiontime < 10) //초기 위치 기록
      {
        qInit[0] = state.motorState[FR_0].q;
        qInit[1] = state.motorState[FR_1].q;
        qInit[2] = state.motorState[FR_2].q;
      }
    //second, move to the origin point of a sine movement with Kp Kd
    //if(motiontime >= 500 && motiontime < 1500){
    if(motiontime >= 10 && motiontime < 400)
      {
        rate_count++;
        double rate = rate_count / 200.0; //needs count to 200
        //Kp[0] = 5.0; Kp[1] = 5.0; Kp[2] = 5.0;
        //Kd[0] = 1.0; Kd[1] = 1.0; Kd[2] = 1.0;
        //Kp와 Kd 설정
        Kp[0] = 20.0;
        Kp[1] = 20.0;
        Kp[2] = 20.0;
        Kd[0] = 2.0;
        Kd[1] = 2.0;
        Kd[2] = 2.0;

        //각 관절의 초기 위치에서 중간 목표 위치까지 선형 이동
        qDes[0] = jointLinearInterpolation(qInit[0], sin_mid_q[0], rate);
        qDes[1] = jointLinearInterpolation(qInit[1], sin_mid_q[1], rate);
        qDes[2] = jointLinearInterpolation(qInit[2], sin_mid_q[2], rate);
      }

    double sin_joint1, sin_joint2;
    //last, do sine wave
    float freq_Hz = 1;
    //float freq_Hz = 5;
    float freq_rad = freq_Hz * 2 * M_PI;
    float t = dt * sin_count;
    if(motiontime >= 400)
    {
      //첫 번째 관절은 중간 목표 위치에 그대로 두고, 두 번째 세 번째 관절은 사인파 형태로 움직임
      sin_count++;
      //sin_joint1 = 0.6 * sin(3 * M_PI * sin_count / 1000.0);
      //sin_joint2 = -0.9 * sin(3 * M_PI * sin_count / 1000.0);
      sin_joint1 = 0.6 * sin(t * freq_rad);
      sin_joint2 = -0.9 * sin(t * freq_rad);
      qDes[0] = sin_mid_q[0];
      qDes[1] = sin_mid_q[1] + sin_joint1;
      qDes[2] = sin_mid_q[2] + sin_joint2;
      //qDes[2] = sin_mid_q[2];
    }

    각 관절에 대해 목표 위치, 속도, P,D 게인, 토크 설정
    cmd.motorCmd[FR_0].q = qDes[0];
    cmd.motorCmd[FR_0].dq = 0;
    cmd.motorCmd[FR_0].Kp = Kp[0];
    cmd.motorCmd[FR_0].Kd = Kd[0];
    cmd.motorCmd[FR_0].tau = -4.0f;

    cmd.motorCmd[FR_1].q = qDes[1];
    cmd.motorCmd[FR_1].dq = 0;
    cmd.motorCmd[FR_1].Kp = Kp[1];
    cmd.motorCmd[FR_1].Kd = Kd[1];
    cmd.motorCmd[FR_1].tau = 0.0f;


    cmd.motorCmd[FR_2].q = qDes[2];
    cmd.motorCmd[FR_2].dq = 0;
    cmd.motorCmd[FR_2].Kp = Kp[2];
    cmd.motorCmd[FR_2].Kd = Kd[2];
    cmd.motorCmd[FR_2].tau = 0.0f;
    //cmd.motorCmd[FR_2].tau = 2 * sin(t * freq_rad);
  }

//if(motiontime > 10){
//  safe.PositionLimit(cmd);
//  safe.PowerProtect(cmd, state, 1);
//  safe.PositionProtect(cmd, state, 0.087);
//}

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




    










    













