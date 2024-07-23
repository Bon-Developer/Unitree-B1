#! /usr/bin/env python3
# ROS를 사용하는 로봇 시스템에 대한 서비스(job) 관리하는 파이썬 스크립트
# 로봇의 시동 설정(upstart) 및 관리를 위해 robot_upstart 라이브러리 사용

import os
import robot_upstart #이거 어디있는지 확인 필요
import rospkg

job_name = "b1_ros"

# 현재 실핼 중인 ROS 서비스(b1_ros) 중지
os.system("sudo service {} stop".format(job_name))

# 서비스 제거
uninstall_job = robot_upstart.Job(name = job_name, rosdistro = os.environ['ROS_DISTRO'])
uninstall_job.uninstall()

# 새 서비스 설치
main_job = robot_upstart.Job(name = job_name,
                             user = 'root',
                             master_uri = os.environ['ROS_MASTER_URI'],
                             rosdistro = os.environ['ROS_DISTRO'],
                             workspace_setup = os.path.join(rospkg.RosPack().get_path('b1_bringup'), 'config', 'setup.bash'))
main_job.add(package = "b1_bringup", filename = "launch/bringup.launch")
main_job.install()

# 서비스 재시작
os.system("sudo systemctl daemon-reload && sudo systemctl start {}".format(job_name)
                            
