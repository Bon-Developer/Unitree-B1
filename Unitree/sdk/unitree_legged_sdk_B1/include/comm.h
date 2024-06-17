/**
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
**/

/**
Header analysis by bon, June 17, 2024
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

  
  //LowState 구조체 정의
  typedef struct
  {
    std::array<uint8_t, 2> head; //패킷의 헤더
    uint8_t levelFlag; //로봇의 통신 수준을 나타내는 플래그
    uint8_t frameReserve; //프레임 예약 필드

    std::array<uint32_t, 2> SN; //시리얼 넘버
    std::array<uint32_t, 2> version; //소프트웨어 또는 하드웨어 버전 정보
    uint16_t bandWidth; //통신 대역폭
    IMU imu; //IMU 데이터 포함
    std::array<MotorState, 20> motorState; //20개의 모터 상태를 나타냄
    BmsState bms; //배터리 관리 시스템(BMS) 상태를 나타냄
    std::array<int16_t, 4> footForce; //reserve, 발의 힘(예약된 필드로 사용되지 않음)
    std::array<int16_t, 4> footForceEst; //reserve, 발의 힘 추정치(예약된 필드로 사용되지 않음
    uint32_t tick; //reference real-time from motion contorller(unit: ms), 모션 컨트롤러의 실시간 기준

    std::array<uint8_t, 40> wirelessRemote; //wireless commands, 무선 명령 데이터 포함
    uint32_t reserve; //예약된 필드로, 나중에 사용될 수 있음

    uint32_t crc; //CRC 체크섬으로 데이터의 무결성을 검증
  } LowState; //low level feedback, LowState 구조체 정의

  //LowCmd 구조체 정의
  typedef struct
  {
    std::array<uint8_t, 2> head; //패킷의 헤더
    uint8_t levelFlag; //로봇의 통신 수준을 나타내는 플래그
    uint8_t frameReserve; //프레임 예약 필드

    std::array<uint32_t, 2> SN; //시리얼 넘버
    std::array<uint32_t, 2> version; //소프트웨어 또는 하드웨어 버전 정보
    uint16_t bandWidth; //통신 대역폭
    std::array<MotorCmd, 20> motorCmd; //20개의 모터 상태를 나타냄
    BmsState bms; //배터리 관리 시스템(BMS) 상태를 나타냄
    std::array<uint8_t, 40> wirelessRemote; //wireless commands, 무선 명령 데이터 포함
    uint32_t reserve; //예약된 필드로, 나중에 사용될 수 있음

    uint32_t crc; //CRC 체크섬으로 데이터의 무결성을 검증
  } LowCmd; //low level control, LowCmd 구조체 정의

  //HighState 구조체 정의
  typedef struct
  {
    std::array<uint8_t, 2> head; //패킷의 헤더
    uint8_t levelFlag; //로봇의 통신 수준을 나타내는 플래그
    uint8_t frameReserve; //프레임 예약 필드

    std::array<uint32_t, 2> SN; //시리얼 넘버
    std::array<uint32_t, 2> version; //소프트웨어 또는 하드웨어 버전 정보
    uint16_t bandWidth; //통신 대역폭
    IMU imu; //IMU 데이터 포함
    std::array<MotorState, 20> motorState; //20개의 모터 상태를 나타냄
    BmsState bms; //배터리 관리 시스템(BMS) 상태를 나타냄
    std::array<int16_t, 4> footForce; //foot force in Z axis, which is parallel to gravity(unit: N), 발에 작용하는 힘을 나타내며 중력 방향(z축)으로의 힘
    std::array<int16_t, 4> footForceEst; //foot force in z axis, which is paralle to gravity(unit: N), 추정된 발의 힘을 나타내며 중력 방향(Z축)으로의 힘
    uint8_t mode; //current mode, more detail in HighCmd comment, 현재 로봇의 모드, HighCmd에 자세하게 설명
    float progress; //reserve, 예약된 필드로 현재 사용되지 않지만 미래 확장을 위해 존재
    uint8_t gaitType; //0.idle, 1.trot, 2.trot running, 3.climb stair, 4.trot obstacle, 보행 유형
    float footRaiseHeight; //(unit: m, default: 0.08m), foot up height while walking, 발을 들어올리는 높이이며 기본 값은 0.08m
    std::array<float, 3> position; //(unit: m), from own odometry in inertial frame, usually drift, 로봇의 위치를 나타내며 로봇 자체 오도메트리에서 얻은 관성 프레임에서의 위치이고, 보통 드리프트가 발생할 수 있음
    float bodyHeight; //(unit: m, default: 0.28m), 로봇 몸체의 높이를 나타내며 기본 값은 0.28m
    std::array<float, 3> velocity; //(unit: m/s), forwardSpeed, sideSpeed, rotateSpeed in body frame, 로봇의 속도를 나타내며 몸체 프레임에서의 전진, 측면, 회전 속도를 나타냄
    float yawSpeed; //(unit: rad/s), rotateSpeed in body frame, 로봇의 회전속도(yaw speed)를 나타내며 몸체 프레임에서의 회전 속도를 나타냄
    std::array<float, 4> rangeObstacle; //reserve, 예약된 필드로 장애물 거리 등을 나타낼 수 있음
    std::array<Cartesian, 4> footPosition2Body; //foot position relative to body, 발의 위치를 몸체 기준으로 나타냄, Cartesian 구조체를 사용하여 각 발의 3D 좌표를 나타냄
    std::array<Cartesian, 4> footSpeed2Body; //foot speed relative to body, 발의 속도를 몸체 기준으로 나타냄, Cartesian 구조체를 사용하여 각 발의 3D 속도를 나타냄
    std::array<uint8_t, 40> wirelessRemote; //wireless commands, 무선 명령 데이터 포함
    uint32_t reserve; //예약된 필드

    uint32_t crc; //CRC 체크섬으로 데이터의 무결성을 검증
  } Highstate; //high level feedback, Highstate 구조체 정의

  //HighCmd 구조체 정의
  typedef struct
  {
    std::array<uint8_t, 2> head; //패킷의 헤더
    uint8_t levelFlag; //로봇의 통신 수준을 나타내는 플래그
    uint8_t frameReserve; //프레임 예약 필드

    std::array<uint32_t, 2> SN; //시리얼 넘버
    std::array<uint32_t, 2> version; //소프트웨어 또는 하드웨어 버전 정보
    uint16_t bandWidth; //통신 대역폭
    uint8_t mode //0. idle(default stand): 기본 서기
                 //1. force stand(controlled by dBodyHeight + ypr): 강제 서기(dBodyHeight + ypr로 제어)
                 //2. target velocity walking(controlled by velocity + yawSpeed): 목표 속도로 걷기(velocity + yawSpeed로 제어)
                 //4. path mode walking(reserve for future release): 경로 모드 걷기(미래 출시를 위한 예약 모드)
                 //5. position stand down: 위치 서기 아래로
                 //6. position stand up: 위치 서기 위로
                 //7. damping mode: 감쇠 모드
                 //9. recovery stand: 복구 서기

    uint8_t gaitType; //0.idle, 1.trot, 2.trot running, 3.climb stair, 4.trot obstacle, 보행 유형
    uint8_t speedLevel; //0.default low speed, 1.medium speed, 2.high speed(during walking, only responde MODE 3), 보행 속도
    float footRaiseHeight; //(unit: m, default: 0.08m), foot up height while walking, 발을 들어올리는 높이이며 기본 값은 0.08m, delta 값의 범위는 [-0.1, 0.15]
    float bodyHeight; //(unit: m, default: 0.28m), 로봇 몸체의 높이를 나타내며 기본 값은 0.28m, delta 값의 범위는 [-0.16, 0.16]
    std::array<float, 2> position; //reserve
    std::array<float, 3> euler; //(unit: rad), roll pitch yaw in stand mode, roll range[-0.3, 0.3], pitch range[-0.3, 0.3], yaw range[-0.6 0.6]
    std::array<float, 2> velocity; //(unit: m/s), forwardSpeed, sideSpeed in body frame, forwardSpeed range[-0.8, 1.2], sideSpeed range[-0.25, 0.25]
    float yawSpeed;
    std::array<float, 2> dComXy; //reserve, desired CoM X,Y 설정할 때 사용할 수 있는 예약 필드
    std::array<float, 2> dstandFootXy; //reserve, desired Foot X,Y 설정할 때 사용할 수 있는 예약 필드
    BmsCmd bms; //배터리 시스템
    std::array<LED, 4> led; //reserve
    std::array<uint8_t, 40> wirelessRemote;
    uint32_t reserve;

    uint32_t crc;
} HighCmd; //high level control, HighCmd 구조체 정의

#pragma pack() //구조체 멤버의 메모리 정렬을 제어하는데 사용, 인수가 없는 경우 구조체의 멤버들이 기본 정렬 방식으로 돌아감

  typedef struct
  {
    unsigned long long TotalCount; //total loop count, 전체 통신 루프가 몇 번 실행되었는지 추적
    unsigned long long SendCount; //total send count, 총 몇 번의 UDP 패킷이 성공적으로 전송되었는지 추적
    unsigned long long RecvCount; //total receive count, 총 몇 번의 UDP 패킷이 성공적으로 수신되었는지 추적
    unsigned long long SendError; //total send error, 전송 과정에서 발생한 오류의 횟수를 추적
    unsigned long long FlagError; //total flag error, 통신 중 발생한 플래그 관련 오류의 횟수를 추적
    unsigned long long RecvCRCError; //total receive CRC error, 수신된 패킷의 CRC 체크에서 발생한 오류의 횟수를 추적
    unsigned long long RecvLoseError; //total lose package count, 통신 중 손실된 패킷의 횟수를 추적
  } UDPState; //UDP communication state, UDPState 구조체 정의
}

#endif
    







