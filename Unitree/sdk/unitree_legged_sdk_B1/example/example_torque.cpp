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
  printf("%d %f %f\n", motiontime, state.motorState[FR_1].q, state.motorState[FR_1].dq); //첫 번째 관절의 각도와 각속도 출력








