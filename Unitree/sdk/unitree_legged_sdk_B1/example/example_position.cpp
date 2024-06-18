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














