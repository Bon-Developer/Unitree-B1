# 240723(Park SangTae): SetMode, SetBodyHeight, SetFootRiaseHeight 추가

#!/user/bin/env python
#credits to: http://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py

from __future__ import print_function #파이썬 2.x와 3.x 간의 호환성을 위해 사용되는 구문

import roslib
import rospy #rospy 라이브러리 가져오기, ROS 노드 초기화 및 ROS 네트워크 내 통신 수행

from geometry_msgs.msg import Twist #로봇 이동 명령(방향 및 속도)
from champ_msgs.msg import Pose as PoseLite #로봇 자세 데이터
from geometry_msgs.msg import Pose as Pose #3차원 공간에서의 위치 방향

#240723
from b1_legged_msgs.msg import SetMode, SetBodyHeight, SetFootRaiseHeight
import tf #ROS에서 좌표 변환 처리하는 라이브러리

import sys, select, termios, tty
import numpy as np

class Teleop:
    def __init__(self):
      self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1) #토픽: cmd_vel, 메시지 타입: Twist

      #base.hpp에 'body_pose/raw', 'body_pose'를 받는 subscriber가 정의되어있지 않음
      self.pose_lite_publisher = rospy.Publisher('body_pose/raw', PoseLite, queue_size = 1) #토픽: body_pose/raw, 메시지 타입: PoseLite
      self.pose_publisher = rospy.Publisher('body_pose', Pose, queue_size = 1) #토픽: body_pose, 메시지 타입: Pose

      #240723    
      # ================ Custom publisher ================
      # base.hpp에서 ros::ServiceServer set_mode_에게 servie를 전송할 client -> mode와 gait type 변경 가능
      self.mode_publisher = rospy.Publisher("/b1_controller/set_mode", SetMode, queue_size = 1)
      self.body_height_publisher = rospy.Publisher("/b1_controller/set_body_height", SetBodyHeight, queue_size = 1)
      self.foot_raise_height_publisher = rospy.Publisher("/b1_controller/set_foot_raise_height", SetFootRaiseHeight, queue_size = 1)
      # ==================================================

      # ================ Custom parameter ================
      self.body_height = rospy.get_param("~body_height", 0.5)
      self.foot_raise_height = rospy.get_param("~foot_raise_height", 0)
      self.gait_type = rospy.get_param("~gait_type", 1)
      # ==================================================

      #Parameters
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
      a/s : increase/decrease body height +- 0.01
      d/f : increase/decrease foot raise height +- 0.01
      y : mode change
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
                'q':(0.1,0.1),
                'z':(-0.1,-0.1),
                'w':(0.1,0),
                'x':(-0.1,0),
                'e':(0,0.1),
                'c':(0,-0.1),
            }
      self.customBindings = {
                'a':(0.01, 0), #bodyHeight + 0.01
                's':(-0.01, 0),
                'd':(0, 0.01), #footRaiseHeight + 0.01
                'f':(0, -0.01)
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
        body_up = 0
        body_down = 0
        foot_up = 0
        foot_down = 0

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
                    self.speed = self.speed + self.speedBindings[key][0] #직진 속도 조정
                    self.turn = self.turn + self.speedBindings[key][1] #회전 속도 조정

                    #15번째 반복마다 안내 메시지를 출력하여 사용자에게 현재 설정 상기
                    print(self.vels(self.speed, self.turn))
                    if (status == 14):
                        print(self.msg)
                    status = (status + 1) % 15

                elif key in self.customBindings.keys():
                    if key in ['a', 's']:
                        req_body = SetBodyHeight()
                        req_body.value = self.customBindings[key][0]
                        print(req_body.value)
                        self.body_height_publisher.publish(req_body)

                    elif key in ['d', 'f']:
                        req_foot = SetFootRaiseHeight()
                        req_foot.value = self.customBindings[key][1]
                        print(req_foot.value)
                        self.foot_raise_height_publisher.publish(req_foot)

                elif key == 'y':
                    self.mode_info = '''
                    # 0. idle, default stand
                    # 1. force stand
                    # 2. target velocity walking
                    # 3. target position walking
                    # 4. path mode walking
                    # 5. position stand down
                    # 6. position stand up
                    # 7. damping mode
                    # 8. recovery stand
                    # 9. backflip
                    # 10. jumpYaw 
                    # 11. straightHand
                    # 12. dance1
                    # 13. dance2
                    '''

                    print(self.mode_info)
                    self.mode = int(input("Select mode: "))

                    self.gait_type_info = "0. idle, 1. trot, 2. trot running, 3. climb start, 4. trot obstacle"
                    print(self.gait_type_info)
                    self.gait_type = int(input("Select gait type : "))

                    if self.gait_type < 0 or self.gait_type > 4:
                       raise ValueError("Invalid gait type. Please select a number between 0 and 4.")
                    elif self.mode < 0 or self.mode > 14:
                       raise ValueError("Invalid mode. Please select a number between 0 and 13.")

                    req_mode = SetMode()
                    req_mode.mode = self.mode
                    req_mode.speed_level = 1
                    req_mode.gait_type = self.gait_type
                    self.mode_publisher.publish(req_mode)

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
            rlist, _, _ = select.select([sys.stdin], [], [], 0.01) #입력이 있을 때까지 대기하는데 최대 0.1초동안 키보드 입력 대기
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
                


        
        
        

        