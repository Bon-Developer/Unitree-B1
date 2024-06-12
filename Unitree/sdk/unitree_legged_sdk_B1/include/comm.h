/**
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
**/

/**
Header analysis by bon, June 12, 2024
**/

//헤더 가드
#ifndef _UNITREE_LEGGED_COMM_H_
#define _UNITREE_LEGGED_COMM_H_

namespace UNITREE_LEGGED_SDK //UNITREE_LEGGED_SDK에 포함되는 모든 내용들
{
  //constexpr: 이 키워드는 상수를 컴파일 시간에 평가되도록 보장, 이를 통해 상수의 사용으로 인한 성능 향상 
  constexpr int HIGHLEVEL = 0xee; //0xee 값의 정수 상수로, 높은 수준의 통신이나 제어 레벨을 나타내는데 사용
  constexpr int LOWLEVEL = 0xff; //0xff 값의 정수 상수로, 낮은 수준의 통신이나 제어 레벨을 나타내는데 사용
  constexpr int TRIGERLEVEL = 0xf0; //0xf0 값의 정수 상수로, 특정 트리거 레벨을 나타내는데 사용
  constexpr double PosStopF = (2.146E+9f); //2.146E+9f 값의 부동 소수점 상수로, 특정 위치 제어 중지 값을 나타내는데 사용, 로봇이 특정 조건에서 멈추도록 하는 값
  constexpr double VelStopF = (16000.0f); //16000.0f 값의 부동 소수점 상수로, 특정 속도 제어 중지 값을 나타내는데 사용, 로봇 속도 제어의 특정 조건에서 멈추도록 하는 값

  //extern: 이 키워드는 변수의 선언을 나타내며, 변수의 정의는 다른 소스 파일에 있다는 것을 의미(변수 공유)
  extern const int HIGH_CMD_LENGTH; //sizeof(HighCmd), HighCmd 구조체의 크기를 나타내는 상수로, 이 값은 구조체의 바이트 단위 크기를 의미
  extern const int HIGH_STATE_LENGTH; //sizeof(HishState), HighState 구조체의 크기를 나타내는 상수로, 이 값은 구조체의 바이트 단위 크기를 의미
  extern const int LOW_CMD_LENGTH; //shorter than sizeof(LowCmd), bytes compressed LowCmd length, LowCmd 구조체의 압축된 바이트 단위 길이를 나타내는 상수로, LowCmd 구조체의 크기보다 작음
  extern const int LOW_STATE_LENGTH; //shorter than sizeof(LowState), bytes compressed LowState length, LowState 구조체의 압축된 바이트 단위 길이를 나타내는 상수로, LowState 구조체의 크기보다 작음

//'BmsCmd'와 'BmsState'라는 두 개의 구조체 정의, 배터리 관리 시스템(BMS)의 명령과 상태를 나타내는데 사용
#pragma pack(1) //구조체 내의 멤버가 1바이트 경계에 맞춰지도록 하여 구조체의 크기를 최소화, 즉, 구조체의 각 멤버는 메모리에서 1바이트 단위로 정렬됨

  //BmsCmd 구조체 정의
  typedef struct
  {
    uint8_t off; //off 0xA5, WARNINGL: it will turn off the battery, 유형: 'uint8_t', 배터리를 끄는 명령으로 0xA5 값을 가지면 배터리가 꺼짐
    std::array<uint8_t, 3> reserve; //유형: 'std::array<uint8_t, 3>', 예약된 필드로, 현재 사용되지 않지만 미래 확장이나 특정 프로토콜 요구 사항을 위해 존재
  } BmsCmd; //BmsCmd 구조체 정의

  //BmsState 구조체 정의
  typedef struct
  {
    uint8_t version_h; //유형: 'uint8_t', BMS 소프트웨어 상위 버전 번호
    uint8_t version_l; //유형: 'uint8_t', BMS 소프트웨어 하위 버전 번호
    uint8_t bms_status; //1 represents normal, others represents abnormal, 유형: 'uint8_t', BMS 상태를 나타내며 1이면 정상, 다른 값이면 비정상 상태를 의미
    uint8_t SOC; //SOC 0-100%, a int range from 0~100, 유형: 'uint8_t', 배터리의 충전 상태를 나타냄
    int32_t current; //mA, negative means discharge, 유형: 'int32_t', 배터리의 현재 전류(mA)를 나타내며 음수이면 방전
    uint16_t cycle; //it represents how many times battery was fully charged and fully discharged, 유형: 'uint16_t', 배터리가 완전히 충전되고 방전된 횟수를 나타냄
    std::array<int8_t, 8> BQ_NTC; //x1 degrees centigrade, 유형: 'std::array<int8_t, 8>', BQ NTC 센서에서 측정한 온도 값 저장, 온도는 섭씨 온도를 나타냄
    std::array<int8_t, 8> MCU_NTC; //x1 degrees centigrade, 유형: 'std::array<int8_t, 8>', MCU NTC 센서에서 측정한 온도 값 저장, 온도는 섭씨 온도를 나타냄
    std::array<uint16_t, 30> cell_vol; //cell voltage mV, 유형: 'std::array<uint16_t, 30>', 각 셀의 전압(mV)을 저장, 최대 30개의 셀 전압을 포함할 수 있음

    //std::array<int8_t, 2> BQ_NTC;
    //std::array<int8_t, 2> MCU_NTC;
    //std::array<uint16_t, 10> cell_vol;
  } BmsState; //BmsState 구조체 정의

  //Cartesian 구조체 정의
  typedef struct
  {
    //3차원 공간에서의 좌표를 나타내는 구조체
    float x;
    float y;
    float z;
  } Cartesian; //Cartesian 구조체 정의

  //IMU 구조체 정의
  typedef struct
  {
    std::array<float, 4> quaternion; //quaternion, normalized, (w,x,y,z), 쿼터니언, 노멀라이즈된 (w,x,y,z) 순서
    std::array<float, 3> gyroscope; //anguler velocity(unit: rad/s) (raw data), 각속도
    std::array<float, 3> accelerometer; //m/(s2) (raw data), 가속도
    std::array<float, 3> rpy; //euler angle(unit: rad), 오일러 각도
    int8_t temperature; //celsius, the IMU's temperature, IMU의 온도
  } IMU; //when under accelerated motion, the attitude of the robot calculated by IMU will drift. IMU 데이터를 나타내는 구조체


  //LED 구조체 정의
  typedef struct
  {
    uint8_t r; //빨간색
    uint8_t g; //초록색
    uint8_t b; //파란색
  } LED; //foot led brightness: 0~255, it's useless in B1, LED의 색상을 나타내는 구조체이며 B1 로봇에서는 사용되지 않음


  //MotorState 구조체 정의
  typedef struct
  {
    uint8_t mode; //motor working mode, 모터 작동 모드
    float q; //current angle(unit: radian), 현재 각도
    float dq; //current velocity(unit: radian/second), 현재 각속도
    float ddq; //current acceleration(unit: radian/second*second), 현재 각가속도
    float tauEst; //current estimated output torque(unit: Nm), 현재 추정 출력 토크
    float q_raw; //current angle(unit: radian), 현재 각도(원시 데이터)
    float dq_raw; //current velocity(unit: radian/second), 현재 각속도(원시 데이터)
    float dqq_raw; //current acceleration(unit: radian/second*second), 현재 각가속도(원시 데이터)
    int8_t temperature; //current temperature(temperature conduction is slow that leads to lag), 현재 온도(온도 전도는 느려서 지연이 발생할 수 있음)
    std::array<uint32_t, 2> reserve; //미래 확장을 위한 예약 필드
  } MotorState; //motor feedback, 모터의 현재 상태를 나타내는 구조체


  //MotorCmd 구조체 정의
  typedef struct
  {
    uint8_t mode; //desired working mode, 원하는 작동 모드
    float q; //desired angle(unit: radian), 원하는 각도
    float dq; //desired velocity(unit: radian/second), 원하는 각속도
    float tau; //desired output torque(unit: Nm), 원하는 출력 토크
    float Kp; //desired position stiffness(unit: Nm/rad), 원하는 위치 강성(P 제어)
    float Kd; //desired velocity stiffness(unit: Nm/(rad/s)), 원하는 속도 강성(D 제어)
    std::array<uint32_t, 3> reserve; //미래 확장을 위한 예약 필드
  } MotorCmd; //motor control, 모터 제어 명령을 나타내는 구조체

  
    











