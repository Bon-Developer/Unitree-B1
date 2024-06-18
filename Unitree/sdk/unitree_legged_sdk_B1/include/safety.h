/**
Copyright(c) 2020, Unitree Robotics.Co.Ltd. All right reserved.
**/

/**
Header analysis by bon, June 17, 2024
**/

#ifndef _UNITREE_LEGGED_SAFETY_H_
#define _UNITREE_LEGGED_SAFETY_H_

#include "comm.h"
#include "quadruped.h"

namespace UNITREE_LEGGED_SDK 
{

class Safety //Safety 클래스 정의
{
  public: 
    Safety(LeggedType type); //생성자, LeggedType을 인자로 받아 로봇의 유형을 설정
    ~Safety(); //소멸자
    void PositionLimit(LowCmd&); //only effect under Low Level control in Position mode, 위치 제한을 설정하는 함수로 low level control의 위치 모드에서만 효과가 있음, LowCmd&: comm.h에서 로봇 명령을 나타내는 구조체
    int PowerProtect(LowCmd&, LowState&, int); //only effect under Low level control, input factor: 1~10, means 10~100% power limit. If you are new, then use 1; if you are familiar, the can try bigger number of even comment this function
                                               //전력 보호를 설정하는 함수, low level control에서만 효과가 있음, (LowCmd&, LowState&): comm.h에서 로봇 명령 및 상태 구조체, int: 전력 제한 계수(1~10)이며 1->10%, 10->100%를 의미
    int PositionProtect(LowCmd&, LowState&, double limit=0.087); //default limit is 5 degree, 각도 제한 설정, double limit: 각도 제한 값으로, 기본값은 0.087 라디안(약 5도)
  
  private:
    int WattLimit, Wcount; //Watt. When limit to 100, you can trigger it with 4 hands shaking, 전력 제한
    double Hip_max, Hip_min, Thigh_max, Thigh_min, Calf_max, Calf_min; //각도 제한 값을 나타내는 변수들, 순서대로 엉덩이 최대/최소각, 허벅지 최대/최소각, 종아리 최대/최소각
};

}

#endif
  
