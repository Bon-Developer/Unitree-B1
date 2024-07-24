/**
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserve.
**/

/**
Header analysis by bon, June 17, 2024
**/

#ifndef _UNITREE_LEGGED_QUADRUPED_H_
#define _UNITREE_LEGGED_QUADRUPED_H_

#include <string> //표준 C++ 문자열 클래스 사용

using namespace std; 

namespace UNITREE_LEGGED_SDK
{
  enum class LeggedType //LeggedType은 로봇의 유형을 정의
  { 
    Aliengo,
    A1,
    Go1,
    B1
  };

//enum class LevelType
//{
//  Low,
//  High,
//};

string VersionSDK(); //SDK 버전을 반환하는 함수
int InitEnvironment(); //memory lock, 환경을 초기화하는 함수, 성공 시 0, 실패 시 다른 값을 번환할 것으로 예상

//definition of each leg and joint
constexpr int FR_ = 0; //leg index, 로봇의 다리 인덱스 정의, constexpr 키워드를 사용하여 컴파일 시간에 평가될 수 있도록 함
constexpr int FL_ = 1;
constexpr int RR_ = 2;
constexpr int RL_ = 3;

constexpr int FR_0 = 0; //joint index, 로봇의 관절 인덱스 정의
constexpr int FR_1 = 1;
constexpr int FR_2 = 2;

constexpr int FL_0 = 3;
constexpr int FL_1 = 4;
constexpr int FL_2 = 5;

constexpr int RR_0 = 6;
constexpr int RR_1 = 7;
constexpr int RR_2 = 8;

constexpr int RL_0 = 9;
constexpr int RL_1 = 10;
constexpr int RL_2 = 11;

}

#endif
