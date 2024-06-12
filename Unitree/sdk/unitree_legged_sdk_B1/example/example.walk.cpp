/****
Copyright (c) 2020, Unitree Robotics.Co.Ltd.All right reserved.
****/

/****
Code analysis by bon, June 12, 2024
****/

#include "unitree_legged_sdk/unitree_legged.sdk.h" //unitree.legged.sdk 헤더 파일 포함하고 있으므로 어떤 헤더파일인지 확인 필요
#include <math.h> 
#include <iostream>
#include <unistd.h>
#include <string.h>

Using namespace UNITREE_LEGGED_SDK; //UNITREE_LEGGED_SDK 라이브러리 사용 -> 여기 내부에 Safety, UDP, HighCmd, HighState 등의 클래스와 구조체 정의

class Custom
{
public:
  //생성자, Custom 클래스의 인스턴스를 초기화
  Custom(unit_8 level) : safe(LeggedType::B1), //'safe' 객체는 'LeggedType:B1'으로 초기화, unsigned 8bit의 'level' 객체 
                         udp(level, 8090, "192.168.123.220", 8082) //'udp' 객체는 'level', 로컬 포트: 8090, 리모트 ip 주소: 192.168.123.220, 리모트 포트: 8082로 초기화
                                                                   //UDP(User Datagram Protocol): 비연결형 서비스 지원하는 전송계층 프로토콜
  {
    udp.InitCmdData(cmd); //'udp' 객채의 명령 데이터(cmd) 초기화, 즉 'cmd' 구조체 초기화
    //udp.print = true; //디버깅을 위해 UDP 통신 내용 출력
  }

  //메소드 선언(멤버 함수)
  void UDPRecv(); //UDP 데이터 수신 메소드
  void UDPSend(); //UDP 데이터 전송 메소드
  Void RobotControl(); //로봇 제어 메소드

  //멤버 변수 선언
  //Safety, UDP, HighCmd, HighState는 UNITREE_LEGGED_SDK에서 제공하는 구조체(클래스)
  Safety safe; //안전 관련 기능 관리 클래스
  UDP udp; //UDP 네트워크 통신 처리 클래스
  HighCmd cmd = {0}; //로봇에 전달할 명령을 담는 클래스, 초기값 0으로 되어 있음
  HighState state = {0}; //로봇 상태 모니터링하고, 제어 시스템에 전달하는 클래스, 초기값 0으로 되어 있음
  int motiontime = 0; //로봇 동작 시간 저장 변수
  float dt = 0.002; //로봇 제어 시간 간격 변수, 0.001~0.01로 설정 가능
}; //여기까지 Custom 클래스 정의

//메소드 정의
//UDP를 통해 데이터 수신
void Custom::UDPRecv()
{
  udp.Recv(); //UDP 클래스의 udp 변수에서 Recv 메소드 호출
}

//UDP를 통해 데이터 전송
void Custom::UDPSend()
{
  udp.Send(); //UDP 클래스의 udp 변수에서 Send 메소드 호출
}

//RobotControl 메소드 정의
{
  motiontime +=2; //motiontime 2씩 증가
  udp.GetRecv(state); //UDP를 통해 수신된 데이터를 state 변수에 저장
  printf("%d %f\n", motiontime, state.imu.rpy[2]); //현재 motiontime과 로봇으 yaw 각도 출력

  //SDK에서 제공하고 있는 cmd 변수들 확인 필요
  cmd.mode = 0; //0: default stand, 1: forced stand, 2: walk Continuoulsy, SDK에서 제공된 cmd 모드
  cmd.gaitType = 0; //보행 패턴
  cmd.speedLevel = 0; //속도 수준
  cmd.footRaiseHeight = 0; //발 높이
  cmd.bodyHeight = 0; //몸체 높이
  cmd.euler[0] = 0; //몸체의 roll 각도
  cmd.euler[1] = 0; //몸체의 pitch 각도
  cmd.euler[2] = 0; //몸체의 yaw 각도
  cmd.velocity[0] = 0.0f; //몸체의 전진 속도
  cmd.velocity[1] = 0.0f; //몸체의 측면 속도
  cmd.yawSpeed = 0.0f; //몸체의 yaw 속도
  cmd.reserve = 0; //예약된 변수
