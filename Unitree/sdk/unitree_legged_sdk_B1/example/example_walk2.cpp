/****
Copyright (c) 2020, Unitree Robotics.Co.Ltd.All right reserved.
****/

/****
Code analysis by bon, July 9, 2024
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

  //동작 예제문
  if(motiontime > 0 && motiontime < 5000)
  {
    cmd.mode = 0; //기본 서기
  }    
  else if(motionitme >= 5000 && motiontime < 15000)
  {
    cmd.mode = 1; //강제 서기
  }
  else if(motionitme >= 15000 && motiontime < 20000)
  {
    cmd.mode = 2; //걷기 모드
    cmd.gaitType = 0; //기본 모드
    cmd.velocity[0] = 0.1; //전진
  }
  else if(motionitme >= 20000 && motiontime < 25000)
  {
    cmd.mode = 2; //걷기 모드
    cmd.gaitType = 0; //기본 모드
    cmd.velocity[0] = -0.1; //후진
  }
  else if(motionitme >= 25000 && motiontime < 30000)
  {
    cmd.mode = 2; //걷기 모드
    cmd.gaitType = 1; //trot 모드
    cmd.velocity[0] = 0.1; //전진
  }
  else if(motionitme >= 30000 && motiontime < 35000)
  {
    cmd.mode = 2; //걷기 모드
    cmd.gaitType = 1; //trot 모드
    cmd.velocity[0] = -0.1; //후진
  }
  else if(motionitme >= 35000 && motiontime < 40000)
  {
    cmd.mode = 2; //걷기 모드
    cmd.gaitType = 2; //trot running 모드
    cmd.velocity[0] = 0.1; //전진
  }
  else if(motionitme >= 40000 && motiontime < 45000)
  {
    cmd.mode = 2; //걷기 모드
    cmd.gaitType = 2; //trot 모드
    cmd.velocity[0] = -0.1; //후진
  }
  else if(motionitme >= 45000 && motiontime < 50000)
  {
    cmd.mode = 2; //걷기 모드
    cmd.gaitType = 3; //climb stair 모드
    cmd.velocity[0] = 0.1; //전진
  }
  else if(motionitme >= 50000 && motiontime < 55000)
  {
    cmd.mode = 2; //걷기 모드
    cmd.gaitType = 3; //climb stair 모드
    cmd.velocity[0] = -0.1; //후진
  }
  else if(motionitme >= 55000 && motiontime < 60000)
  {
    cmd.mode = 2; //걷기 모드
    cmd.gaitType = 4; //trot obstacle 모드
    cmd.velocity[0] = 0.1; //전진
  }
  else if(motionitme >= 60000 && motiontime < 65000)
  {
    cmd.mode = 2; //걷기 모드
    cmd.gaitType = 4; //trot obstacle 모드
    cmd.velocity[0] = -0.1; //후진
  }
  else if(motionitme >= 65000 && motiontime < 70000)
  {
    cmd.mode = 1; //강제 서기
  }
  else
  {
    cmd.mode = 0; //else문이니 아무것도 안하거나 이전 상태 유지하는 것인지 확인 필요
  }

  udp.SetSend(cmd); //설정된 명령을 UDP를 통해 전송
}

//main 문
int main(void)
{
  std::cout<<"Communication level is set to HIGH-level."<<std::endl
           <<"WARNING: Make sure the robot is standing on the ground."<<std::endl
           <<"Press Enter to continue..."<<std::endl;
  std::cin.ignore(); //Enter 키 입력 기다리기

  Custom custom(HIGHLEVEL); //'Custom' 클래스의 객체 'custom' 생성, 'HIGHLEVEL'은 로봇과의 통신 수준을 설정하는 상수
  InitEnvironment(); //로봇 제어를 위한 환경 초기화

  //LoopFunc 클래스의 객체를 생성하여 제어 루프와 UDP 송수신 루프 설정
  //boost::bind는 메소드와 객체를 결합하여 루프에서 실행할 수 있도록 함
  LoopFunc loop_control("control_loop", custom.dt, boost::bind(&Custom::RobotControl, &custom)); //로봇 제어 루프 설정, dt 간격으로 실행, Custom::RobotControl 메소드 호출
  LoopFunc loop_udpSend("udp_send", custom.dt, 3, boost::bind(&Custom::UDPSend, &custom)); //UDP 데이터 전송 루프 설정, dt 간격으로 실행, Custom::UDPSend 메소드 3회 호출 
  LoopFunc loop_udpRecv("udp_recv", custom.dt, 3, boost::blind(&Custom::UDPRecv, &custom)); //UDP 데이터 수신 루프 설정, dt 간격으로 실행, Custom::UDPRecv 메소드 3회 호출

  //루프 시작
  loop_udpSend.start();
  loop_udpRecv.start();
  loop_control.start();

  while(1)
  {
    sleep(10); //무한 루프를 실행하여 프로그램 종료되지 않음, 루프는 너무 자주 실행되지 않도록 10 간격으로 대기(단위 확인 필요)
  };

  return 0; //main 함수가 0을 반환하여 정상적으로 종료됨을 나타냄
}
