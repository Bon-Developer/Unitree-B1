#!/user/bin/env python
#credits to: http://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py

from __future__ import print_function #파이썬 2.x와 3.x 간의 호환성을 위해 사용되는 구문

import roslib; roslib.load_manifest('champ_teleop') #roslib 라이브러리 가져오기, champ_teleop 패키지 로드
import rospy #rospy 라이브러리 가져오기, ROS 노드 초기화 및 ROS 네트워크 내 통신 수행

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from champ_masgs.msg import Pose as PoseLite
from geometry_msgs.msg import Pose as Pose
import tf
