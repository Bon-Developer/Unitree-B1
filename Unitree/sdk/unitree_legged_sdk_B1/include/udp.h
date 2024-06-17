/**
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
**/

/**
Header analysis by bon, June 17, 2024
**/

#ifndef _UNITREE_LEGGED_UDP_H_
#define _UNITREE_LEGGED_UDP_H_

#include "comm.h"
#include "quadruped.h"
#include <pthread.h>
#include <stdint.h>

/*
  UDP critical configuration:

  1. initiativeDisconnect: if need disconnection after connected, another ip/port can access after disconnection

      /---block  will block till data come
  2. recvType  ----block_timeout  will block till data come or timeout
      \---non block  if no data will reture immediately

      /---Y  ip/port will be set later
  3. setIpPort:
      \---N  ip/port not specified, as a server wait for connect
*/

namespace UNITREE_LEGGED_SDK
{
  constexpr int UDP_CLIENT_PORT = 8080; //local port, 로컬 포트를 나타내는 상수, 기본값 8080
  constexpr int UDP_SERVER_PORT = 8007; //target port, 목표 포트를 나타내는 상수, 기본값 8007
  constexpr char UDP_SERVER_IP_BASIC[] = "192.168.123.10"; //target IP address, 기본 모드 IP 주소를 나타내는 상수
  constexpr char UDP_SERVER_IP_SPORT[] = "192.168.123.161"; //target IP address, 스포츠 모드 IP 주소를 나타내는 상수

  typedef enum
  {
    nonBlock = 0x00, //비차단 모드
    block = 0x01, //차단 모드
    blockTimetout = 0x02 = 0x02, //타임 아웃을 사용하는 차단 모드
  } RecvEnum; //UDP 수신 동작을 제어하는 구조체

//Notice: User defined data(like struct) should add crc(4Byte) at the end.
  class UDP
  {
    public:
    //UDP(uint_t level, LevelType highControl = LevelType::Low); //unitree default IP and Port
    //기본 생성자
    UDP(uint8_t level, uint16_t localPort, const char *targetIP, uint16_t targetPort); //udp use default length according to level, 로컬 포트, 목표 IP 주소 및 목표 포트를 사용하여 UDP 객체 초기화
    //추가 생성자
    UDP(uint16_t localPort, const char *targetIP, uint16_t targetPort, int sendLegnth, int recvLength, bool initiativeDisconnect = false, RecvEnum recvType = RecvEnum::nonBlock); //송신 및 수신 길이와 추가 옵션을 포함한 UDP 객체 초기화
    //또 다른 생성자
    UDP(uint16_t localPort, int sendLength, int recvLength, bool initiativeDisconnect = false, RecvEnum recvType = RecvEnum::nonBlock, bool setIpPort = false); //목표 IP 및 포트를 설정할 필요 없이 UDP 객체 초기화
    ~UDP(); //소멸자

    void SetIpPort(const char *targetIP, uint16_t targetPort); //if not indicated at constructor function, 목표 IP 주소와 포트를 설정
    void SetRecvTimeout(int time); //use in RecvEnum::blockTimeout(unit:ms) //수신 타임아웃을 설정

    void SetDisconnectTime(float callback_dt, float disconnectTime); //initiativeDisconnect = true, disconnect for another IP to connect, 연결 해제 시간 설정
    void SetAccessibleTime(float callback_dt, float accessibleTime); //check if can access data, 데이터 접근 가능 시간 설정

    int Send(); //송신 메소드, 데이터 전송
    int Recv(); //directly save in buffer, 수신 메소드, 데이터를 수신하여 버퍼에 직접 저장

    void InitCmdData(HighCmd &cmd); //HighCmd 명령 데이터 초기화
    void InitCmdData(LowCmd &cmd); //LowCmd 명령 데이터 초기화
    //전송할 데이터 설정
    int SetSend(char *);
    int SetSend(HighCmd &);
    int SetSend(LowCmd &);

    //수신할 데이터 설정
    void GetRecv(char *);
    void GetRecv(HighState &);
    void GetRecv(LowState &);

    UDPState udpState; //UDP 통신 상태를 나타냄
    char *targetIP; //목표 IP 주소를 나타냄
    uint16_t targetPort; //목표 포트를 나타냄
    char *localIP; //로컬 IP 주소를 나타냄
    uint16_t localPort; //로컬 포트를 나타냄
    bool accessible = false; //can access or not, 데이터 접근 가능 여부를 나타냄
    bool print = false; //출력 여부를 나타냄

  private:
    void init(uint16_t loaclPort, const char *targetIP=NULL, uint16_t targetPort = 0); 
    //멤버 변수 정의
    int sockFd; //소켓 파일 디스크립터
    bool connected; //udp works with connect() fuction, rather than server mode;, 연결 여부를 나타냄
    int sendLength; //송신 데이터 길이를 나타냄
    int recvLength; //수신 데이터 길이를 나타냄
    int lose_recv; //수신 손실을 나타냄

    char *recvBuf; //수신 버퍼를 나타냄
    char *recvAvaliable; //사용할 수 있는 수신 버퍼를 나타냄
    char *sendBuf; //송신 버퍼를 나타냄
    pthread_mutex_t sendMutex; //송신 뮤텍스를 나타냄
    pthread_mutex_t recvMutex; //수신 뮤텍스를 나타냄
    pthread_mutex_t udpMutex; //UDP 뮤텍스를 나타냄

    bool nonblock = true; //비차단 모드 여부를 나타냄
    int blockTimeout = -1; //use time out method or not(unit: ms), 타임아웃 시간을 나타냄
    bool initiativeDisconnect = false; //연결 해제 여부를 나타냄
};

}

#endif



    
    






















