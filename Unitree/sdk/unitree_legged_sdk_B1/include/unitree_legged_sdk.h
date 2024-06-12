/**
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
**/

/**
Analysis by bon, June 12, 2024
**/

//헤더 가드: 헤어 파일이 여러번 포함되는 것을 방지
#ifndef _UNITREE_LEGGED_SDK_H_ //_UNITREE_LEGGED_SDK_H_가 정의되지 않은 경우에만 아래 코드 포함
#define _UNITREE_LEGGED_SDK_H_ //_UNITREE_LEGGED_SDK_H_를 정의

//이걸 다 포함 시키고 있는 헤더 파일
#include "comm.h" //통신 관련 헤더 파일
#include "safety.h" //완전 관련 헤더 파일
#include "udp.h" //UDP 통신 관련 헤더 파일
#include "loop.h" //루프 제어 관련 헤더 파일
#include "quadruped.h" //4족 보행 로봇 관련 헤더 파일
#include "joystick.h" //조이스틱 제어 관련 헤더 파일
#include <boost/bind.hpp> //Boost 라이브러리의 'bind' 기능 포함

#define UT UNITREE_LEGGED_SDK //'UNITREE_LEGGED_SDK' 네임스페이스를 'UT'로 사용할 수 있게 정의, 'UNITREE_LEGGED_SDK::' 대신 'UT::' 사용 가능

#endif //헤더 가드 닫기(#ifndef와 #define의 쌍을 닫음)
