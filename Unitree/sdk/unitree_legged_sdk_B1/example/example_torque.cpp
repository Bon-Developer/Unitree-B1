/**
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All reserved.
**/

/**
Code anlysis by bon, June 24, 2024
**/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>

using namespace UNITREE_LEGGED_SDK; //헤더파일에서 sdk 접근

class Custom
{
  public:
    Custom(uint8_t level): //클래스 생성자
      safe(LeggedType::B1), //B1 타입의 로봇 안전 객체 초기화
      udp(level, 8090, "192.168.123.10", 8007){ //UDP 객체 초기화, 객체: 'level', 포트: 8090, IP 주소: 192.168.123.10, 포트: 8007
      udp.InitCmdData(cmd); //명령 데이터 초기화
    }

    //멤버 함수 정의
    void UDPSend();
    void UDPRecv();
    void RobotControl();

    //멤버 변수 정의
    Safety safe;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};
    int motiontime = 0;
    float dt = 0.002; //0.001 ~ 0.01
    int sin_count = 0;
};

void Custom::UDPRecv()
{
  udp.Recv();
}

void Custon::UDPSend()
{
  udp.Send();
}

void Custom::RobotControl()
{
  motiontime++; //제어 루프의 진행 추적
  udp.GetRecv(state); //현재 로봇의 상태 수신
  printf("%d %f %f\n", motiontime, state.motorState[FR_1].q, state.motorState[FR_1].dq); //FR_1 관절의 각도와 각속도 출력

  //gravity compensation
  cmd.motorCmd[FR_0].tau = -5.0f; //FR_0 관절에 중력 보상 토크 적용
  //cmd.motorCmd[FL_0].tau = +0.65f;
  //cmd.motorCmd[RR_0].tau = -0.65f;
  //cmd.motorCmd[RL_0].tau = +0.65f;

  //float freq_Hz = 1;
  //float freq_Hz = 2;
  //float freq_Hz = 5;
  //float freq_rad = freq_Hz * 2 * M_PI;
  //float t = dt * sin_count;

  if(motiontime >= 500)
  {
    sin_count++; //제어 주기
    float torque = (0 - state.motorState[FR_1].q) * 10.0f + (0 - state.motorState[FR_1].dq) * 1.0f; //관절 현재 각도와 각속도를 이용하여 토크 계산
    //float torque = (0 - state.motorState[FR_1].q) * 20.0f + (0 - state.motorState[FR_1].dq) * 2.0f;
    //float torque = (0 - state.motorState[FR_1].q) * 40.0f + (0 - state.motorState[FR_1].dq) * 2.0f;
    //float torque = 2 * sin(t * freq_rad);

    //계산된 토크가 일정 범위를 넘지 않도록 클리핑
    if(torque > 5.0f) torque = 5.0f;
    if(torque < -5.0f) torque = -5.0f;
    //if(torque > 15.0f) torque = 15.0f;
    //if(torque < -15.0f) torque = -15.0f;

    cmd.motorCmd[FR_1].q = PosStopF;
    









