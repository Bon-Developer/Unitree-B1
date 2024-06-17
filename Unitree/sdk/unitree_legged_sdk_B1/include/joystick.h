/**
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
**/

/**
Header analysis by bon, June 17, 2024
**/

#ifndef _UNITREE_LEGGED_JOYSTICK_H_
#define _UNITREE_LEGGED_JOYSTICK_H_

#include <stdint.h> 

//16bit 데이터를 저장할 수 있는 공용체 정의
typedef union
{
  struct //각 버튼의 상태를 개별 비트 필드로 표현
  {
    //비트 필드: 각 버튼이 1비트로 표현되며, 이는 버튼이 눌렸는지(1) 또는 눌리지 않았는지(0)를 나타냄
    uint8_t R1 :1;
    uint8_t L1 :1;
    uint8_t start :1;
    uint8_t select :1;
    uint8_t R2 :1;
    uint8_t L2 :1;
    uint8_t F1 :1;
    uint8_t F2 :1;
    uint8_t A :1;
    uint8_t B :1;
    uint8_t X :1;
    uint8_t Y :1;
    uint8_t up :1;
    uint8_t right :1;
    uint8_t down :1;
    uint8_t left :1;
  } components;
  uint16_t value;
} xKeySwitchUnion;

//40Byte(noew used 24Byte)
//컨트롤러의 버튼과 조이스틱 데이터를 저장하는 구조체
typedef struct
  {
    uint8_t head[2]; //2바이트 헤더 필드, 패킷의 시작을 나타냄
    xKeySwitchUnion btn; //xKeySwitchUnion 타입의 버튼 상태, 16비트로 표현되는 모든 버튼의 상태를 포함
    float lx; //왼쪽 조이스틱의 x축 위치를 나타냄
    float rx; //오른쪽 조이스틱의 x축 위치를 나타냄
    float ry; //오른쪽 조이스틱의 y축 위치를 나타냄
    float L2; //L2 버튼의 아날로그 입력 값을 나타냄
    float ly; //왼쪽 조이스틱의 y축 위치를 나타냄

    uint8_t idle[16]; //나머지 16바이트는 idle 필드로 예약
  } xRockerBtnDataStruct;

#endif // _UNITREE_LEGGED_JOYSTICK_H_
    






