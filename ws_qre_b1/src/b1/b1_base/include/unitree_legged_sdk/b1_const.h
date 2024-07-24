/**
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
**/

/**
Header anlaysis by bon June 17, 2024
**/

#ifndef B1_CONST_H
#define B1_CONST_H
namespace UNITREE_LEGGED_SDK
{
  //엉덩이 관절 최대/최소 각도[radian]
  constexpr double b1_Hip_max = 0.75; //42.97도
  constexpr double b1_Hip_min = -0.75; //-42.97도
  //허벅지 관절 최대/최소 각도[radian]
  constexpr double b1_Thigh_max = 3.5; //200.54도
  constexpr double b1_Thigh_min = -1.0; //-57.3도
  //종아리 관절 최대/최소 각도[radian]
  constexpr double b1_Calf_max = -0.6; //-34.38도
  constexpr double b1_Calf_min = -2.6; //-148.97도
}
#endif
