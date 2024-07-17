#!/user/bin/env python
#credits to: http://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py

from __future__ import print_function #파이썬 2.x와 3.x 간의 호환성을 위해 사용되는 구문

import roslib
import roslib.load_manifest('champ_teleop') #roslib 라이브러리 가져오기, champ_teleop 패키지 로드
import rospy #rospy 라이브러리 가져오기, ROS 노드 초기화 및 ROS 네트워크 내 통신 수행

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
      self.swing_height = rospy.get_param("gait/swing_height", 0) #파라미터 이름: gait/swing_hieght, 파라미터 값 가져옴. 기본값 0
      self.nominal_height = rospy.get_param("gait/nominal_height", 0) #파라미터 이름: gait/nominal_height, 파라미터 값 가져옴. 기본값 0

      self.speed = rospy.get_param("~speed", 0.1) #기본 속도 0.1m/s
      self.turn = rospy.get_param("~turn", 0.1) #기본 속도 0.1m/s

      
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


    def poll_keys(self): #키보드 입력을 폴링하여 이동 명령 처리, termois와 tty 모듈 사용하여 터미널 설정 제어 및 비차단 모드로 키보드 입력 읽어옴
        self.settings = termios.tcgetattr(sys.stdin) #현재 터미널의 설정을 가져와 self.settings에 저장

        #변수 초기화
        x = 0
        y = 0
        z = 0
        th = 0
        roll = 0
        pitch = 0
        yaw = 0
        status = 0
        cmd_attempts = 0

        try:
            print(self.msg) #위 키보드 터미널 메시지 알려줌
            print(self.vels(self.speed, self.turn)) #현재 설정된 속도와 회전 속도 반환

            
            while not rospy.is_shutdown():
                key = self.getKey() #단일 키 입력을 읽어오는 메서드, 입력이 없으면 빈 문자열 반환
                if key in self.velocityBindings.keys():
                    x = self.velocityBindings[key][0]
                    y = self.velocityBindings[key][1]
                    z = self.velocityBindings[key][2]
                    th = self.velocityBindings[key][3]


                    if cmd_attempts > 1:
                        twist = Twist()
                        twist.linear.x = x * self.speed
                        twist.linear.y = y * self.speed
                        twist.linear.z = z * self.speed
                        twist.angular.x = 0
                        twist.angular.y = 0
                        twist.angular.z = th * self.turn
                        self.velocity_publisher.publish(twist) #Twist 메시지를 cmd_vel 토픽에 발행
    
                    cmd_attempts += 1

                elif key in self.speedBindings.keys():
                    self.speed = self.speed * self.speedBindings[key][0] #직진 속도 조정
                    self.turn = self.turn * self.speedBindings[key][1] #회전 속도 조정

                    #15번째 반복마다 안내 메시지를 출력하여 사용자에게 현재 설정 상기
                    print(self.vels(self.speed, self.turn))
                    if (status == 14):
                        print(self.msg)
                    status = (status + 1) % 15

                else:
                    cmd_attempts = 0 #아무 명령이 없으면 0으로 초기화
                    if (key == '\x03'): #ctrl + C가 입력되면 루프 종료
                        break

        #예외 발생 시 예외 메시지 출력
        except Exception as e:
            print(e)
        #새로운 Twist 메시지를 생성하고, 모든 값을 0으로 설정하여 로봇 정지
        finally:
            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            self.velocity_publisher.publisher(twist)

            termois.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings) #터미널 설정을 원래 상태로 복원

        def getKey(self):
            tty.setraw(sys.stdin.fileno()) #터미널을 raw 모드로 설정
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1) #입력이 있을 때까지 대기하는데 최대 0.1초동안 키보드 입력 대기
            if rlist:
                key = sys.stdin.read(1) #키보드 입력 한 글자 읽어옴
            else:
                key = ''
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings) #터미널을 원래 상태로 복원하여 정상적인 입력 모드로 변경
            return key

        #현재 설정된 속도와 회전 속도를 형식화된 문자열로 반환
        def vels(self, speed, turn):
            return "currently: \tspeed %s\tturn %s " % (speed, turn)
            
        #주어진 입력 값 x를 지정된 입력 범위에서 출력 범위로 변환
        def map(self, x, in_min, in_max, out_min, out_max):
            return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

if __name__ == "__main__":
    rospy.init_node('b1_teleop')
    teleop = Teleop()
                


        
        
        

        
