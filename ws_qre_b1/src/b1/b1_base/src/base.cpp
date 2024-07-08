# include "b1_base/base.hpp"

using namespace qre;
Base::Base(UT::HighCmd cmd, UT::HighState state, ros::NodeHandle *nh, ros::NodeHandle *p_nh): robot_safety_(UT::LeggedType::B1)
  nh_ = nh;
  robot_high_cmd.mode            = 2;
  robot_high_cmd.gaitType        = 1;
  robot_high_cmd.velocity        = {0.0};
  robot_high_cmd.position        = {0.0};
  robot_high_cmd.yawSpeed        = 0.0;
  robot_high_cmd.euler           = {0.0};
  robot_high_cmd.bodyHeight      = 0.0;
  robot_high_cmd.footRaiseHeight = 0.0;
