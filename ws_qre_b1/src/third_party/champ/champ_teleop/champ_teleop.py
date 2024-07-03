#!/user/bin/env python
#credits to: http://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py

from __future__ import print_function #파이썬 2.x와 3.x 간의 호환성을 위해 사용되는 구문

import roslib; roslib.load_manifest('champ_teleop') #roslib 라이브러리 가져오기, champ_teleop 패키지 로드
import rospy #rospy 라이브러리 가져오기, ROS 노드 초기화 및 ROS 네트워크 내 통신 수행

from sensor_msgs.msg import Joy #조이스틱 입력 송수신
from geometry_msgs.msg import Twist #로봇 이동 명령(방향 및 속도)
from champ_masgs.msg import Pose as PoseLite #로봇 자세 데이터
from geometry_msgs.msg import Pose as Pose #3차원 공간에서의 위치 방향 
import tf #ROS에서 좌표 변환 처리하는 라이브러리

import sys, select, termios, tty
import numpy as np

class Teleop:
    def __init__(self):
      self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1) #토픽: cmd_vel, 메시지 타입: Twist
      self.pose_lite_publisher = rospy.Publisher('body_pose/raw', PoseLite, queue_size = 1) #토픽: body_pose/raw, 메시지 타입: PoseLite
      self.pose_publisher = rospy.Publisher('body_pose', Pose, queue_size = 1) #토픽: body_pose, 메시지 타입: Pose
      self.joy_subscriber = rospy.Subscriber('joy', Joy, self.joy_callback) #토픽: joy, 메시지 타입: Joy, joy 토픽에 새로운 메시지가 도착할 때마다 콜백 함수 호출
      self.swing_height = rospy.get_param("gait/swing_height", 0) #파라미터 이름: gait/swing_hieght, 파라미터 값 가져옴. 기본값 0
      self.nominal_height = rospy.get_param("gait/nominal_height", 0) #파라미터 이름: gait/nominal_height, 파라미터 값 가져옴. 기본값 0

      self.speed = rospy.get_param("~speed", 0.5)
      self.turn = rospy.get_param("~turn", 0.5)

      #
      self.msg = """
      Reading from the keyboard and Publishing to Twist!
      ---------------------------
      Moving around:
      u    i    o
      j    k    l
      m    ,    .
      For Holonomic mode (strafing), hold down the shift key: 
      ---------------------------
      U    I    O
      J    K    L
      M    <    >
      t : up (+z)
      b : down (-z)
      anything else : stop
      q/z : increase/decrease max speeds by 10%
      w/x : increase/decrease only linear speed by 10%
      e/c : increase/decrease only angular speed by 10%
      CTRL-C to quit
                """

      self.velocityBindings = {
                'i':(1,0,0,0),
                'o':(1,0,0,-1),
                'j':(0,0,0,1),
                'l':(0,0,0,-1),
                'u':(1,0,0,1),
                ',':(-1,0,0,0),
                '.':(-1,0,0,1),
                'm':(-1,0,0,-1),
                'O':(1,-1,0,0),
                'I':(1,0,0,0),
                'J':(0,1,0,0),
                'L':(0,-1,0,0),
                'U':(1,1,0,0),
                '<':(-1,0,0,0),
                '>':(-1,-1,0,0),
                'M':(-1,1,0,0),
                'v':(0,0,1,0),
                'n':(0,0,-1,0),
            }

      self.poseBindings = {
                'f':(-1,0,0,0),
                'h':(1,0,0,0),
                't':(0,1,0,0),
                'b':(0,-1,0,0),
                'r':(0,0,1,0),
                'y':(0,0,-1,0),
            }

      self.speedBindings = {
                'q':(1.1,1.1),
                'z':(.9,.9),
                'w':(1.1,1),
                'x':(.9,1),
                'e':(1,1.1),
                'c':(1,.9),
            }
        
      self.poll_keys()

