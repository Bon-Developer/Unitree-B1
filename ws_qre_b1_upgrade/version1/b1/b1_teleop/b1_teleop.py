#!/usr/bin/env python
#-*- coding:utf-8 -*-
#credits to: https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py

from __future__ import print_function

import roslib; roslib.load_manifest('champ_teleop')
import rospy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from champ_msgs.msg import Pose as PoseLite
from geometry_msgs.msg import Pose as Pose
from b1_legged_msgs.msg import SetMode, SetBodyHeight, SetFootRaiseHeight
import tf

import sys, select, termios, tty
import numpy as np

class Teleop:
    def __init__(self):
        
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

        # base.hpp에 'body_pose/raw', 'body_pose'를 받는 subscriber가 정의되어있지 않다.
        self.pose_lite_publisher = rospy.Publisher('body_pose/raw', PoseLite, queue_size = 1)
        self.pose_publisher = rospy.Publisher('body_pose', Pose, queue_size = 1)
        self.joy_subscriber = rospy.Subscriber('joy', Joy, self.joy_callback)

        # * ============== Custom handle ===================
        # base.hpp에서 ros::ServiceServer set_mode_에게 service를 전송할 client -> mode와 gait_type변경 가능
        self.mode_publisher = rospy.Publisher("/b1_controller/set_mode", SetMode, queue_size = 1)
        self.body_height_publisher = rospy.Publisher("/b1_controller/set_body_height", SetBodyHeight, queue_size = 1)
        self.foot_raise_height_publisher = rospy.Publisher("/b1_controller/set_foot_raise_height", SetFootRaiseHeight, queue_size = 1)
        # * ==============================================


        # Parameters
        self.swing_height = rospy.get_param("gait/swing_height", 0)
        self.nominal_height = rospy.get_param("gait/nominal_height", 0)

        self.speed = rospy.get_param("~speed", 0.5)
        self.turn = rospy.get_param("~turn", 1.0)


        # * ============== Custom Parameter =================
        self.body_height = rospy.get_param("body_height", 0.5) # ! 바꿈
        self.foot_raise_height = rospy.get_param("~foot_raise_height", 0.08)
        self.gait_type = rospy.get_param("~gait_type", 1)
        # * =================================================


        self.msg =  """
                    Reading from the keyboard  and Publishing to Twist!
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
                    d/f : increase/decrease body height +- 0.01
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

        self.speedBindings={
                'q':(1.1, 1.1),
                'z':(.9, .9),
                'w':(1.1,1),
                'x':(.9,1),
                'e':(1,1.1),
                'c':(1,.9),
            }
        
        self.customBindings={
                'a' : ( 0.01, 0), # bodyHeight + 0.01
                's' : (-0.01, 0),
                'd' : (0,  0.01), # footRaiseHeight + 0.01
                'f' : (0, -0.01)            
            }
        
        self.poll_keys()

    def joy_callback(self, data):
        twist = Twist()
        twist.linear.x = data.axes[1] * self.speed
        twist.linear.y = data.buttons[4] * data.axes[0] * self.speed
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = (not data.buttons[4]) * data.axes[0] * self.turn
        self.velocity_publisher.publish(twist)

        body_pose_lite = PoseLite()
        body_pose_lite.x = 0
        body_pose_lite.y = 0
        body_pose_lite.roll = (not data.buttons[5]) *-data.axes[3] * 0.349066
        body_pose_lite.pitch = data.axes[4] * 0.174533
        body_pose_lite.yaw = data.buttons[5] * data.axes[3] * 0.436332
        if data.axes[5] < 0:
            body_pose_lite.z = data.axes[5] * 0.5

        self.pose_lite_publisher.publish(body_pose_lite)

        body_pose = Pose()
        body_pose.position.z = body_pose_lite.z

        quaternion = tf.transformations.quaternion_from_euler(body_pose_lite.roll, body_pose_lite.pitch, body_pose_lite.yaw)
        body_pose.orientation.x = quaternion[0]
        body_pose.orientation.y = quaternion[1]
        body_pose.orientation.z = quaternion[2]
        body_pose.orientation.w = quaternion[3]

        self.pose_publisher.publish(body_pose)

    def poll_keys(self):
        self.settings = termios.tcgetattr(sys.stdin) # ? 터미널의 설정을 저장하는 코드

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
            print(self.msg)
            print(self.vels( self.speed, self.turn))
            
            while not rospy.is_shutdown():
                key = self.getKey() # 여기서 key를 감지
                if key in self.velocityBindings.keys():
                    x = self.velocityBindings[key][0]
                    y = self.velocityBindings[key][1]
                    z = self.velocityBindings[key][2]
                    th = self.velocityBindings[key][3]
                    
                    if cmd_attempts > 1:
                        twist = Twist()
                        twist.linear.x = x *self.speed
                        twist.linear.y = y * self.speed
                        twist.linear.z = z * self.speed
                        twist.angular.x = 0
                        twist.angular.y = 0
                        twist.angular.z = th * self.turn
                        self.velocity_publisher.publish(twist)
                    cmd_attempts += 1
                    
                # speedBindgs키를 눌러도 velocityBindings키를 누르지 않는 한 움직이지 않는 것이 좋네.
                elif key in self.speedBindings.keys():
                    self.speed = self.speed * self.speedBindings[key][0]
                    self.turn = self.turn * self.speedBindings[key][1]
                    
                    print(self.vels(self.speed, self.turn))
                    if (status == 14): # ? 이건 뭐고?
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
                    # 1. force stand (controlled by dBodyHeight + ypr)
                    # 2. target velocity walking (controlled by velocity + yawSpeed)
                    # 3. target position walking (controlled by position + ypr[0])
                    # 4. path mode walking (reserve for future release)
                    # 5. position stand down. 
                    # 6. position stand up 
                    # 7. damping mode 
                    # 8. recovery stand
                    # 9. backflip
                    # 10. jumpYaw
                    # 11. straightHand
                    # 12. dance1
                    # 13. dance2"
                    '''

                    print(self.mode_info)
                    self.mode = int(input("Select mode : "))

                    self.gait_type_info = "0.idle  1.trot  2.trot running  3.climb stair  4.trot obstacle"
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
                    cmd_attempts = 0
                    if (key == '\x03'): # ctrl + C를 누르면 종료
                        break

        except Exception as e:
            print(e)

        finally: # 예외 발생여부에 상관없이 항상 수행된다.  
            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            self.velocity_publisher.publish(twist)

            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def vels(self, speed, turn):
        return "currently:\tspeed %s\tturn %s " % (speed,turn)

    def map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

if __name__ == "__main__":
    rospy.init_node('champ_teleop')
    teleop = Teleop()
