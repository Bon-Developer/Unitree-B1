# Quadruped Robotics B1 ROS Wrapper

> [B1 Docs](https://www.docs.quadruped.de/projects/B1/html/index.html)

## Requirements

Current version of the [`b1_base`](b1/b1_base) is built upon `unitree_legged_sdk-3.8.3`.

The requirements are as follows:

### Sport Mode

```bash
Legged_sport    >= v3.24
firmware H0.1.7 >= v0.1.35
         H0.1.9 >= v0.1.35
```

## Installation

- Clone the repository into your ROS1 workspace.

```
git clone https://github.com/MYBOTSHOP/qre_B1.git
```

- Go to your ROS workspace and install ROS dependencies

```
rosdep install --from-paths src --ignore-src -r -y
```

- Install B1 Navigation Dependencies, first provide permissions and then execute

```
sudo chmod +x install.bash && ./install.bash
```

- For B1 Navigation, you would additionally require the ROS repository of [yocs_velocity_smoother](https://github.com/yujinrobot/yujin_ocs)

- Build and source your ROS workspace

```
catkin build 
source devel/setup.bash
```

## LAN Static Connection


To create a static connection in your own PC, in Ubuntu go to Settings → Network then click on ``+`` and create a new connection.

1. The first task is to go to **IPv4** and change the connection to **manual**.

2. The second task is to put the **Address** IP as **192.168.123.51** and the **Netmask** as **24**.


3. Click save and restart your network. 


4. After a successful connection let's check the host's local IP by typing in the Host PC's terminal.

```
ifconfig
```

5. This should show the host IP which was assigned in the above step. Now its time to check if we can ping the robot or not, to do so type in your host pc

```
ping 192.168.123.220
```

6. After a successful ping, it's time to access the robot. To access the robot you can type the following command:

```
ssh -X unitree@192.168.123.220
```

The password for all is 

```
123
```

## Quick start

Provided that you connected via LAN, you can start and use the robot via:

```
sudo su
source <your_workspace>/devel/setup.bash
roslaunch b1_bringup bringup.launch
```

You can then teleop via

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

To visualize in RVIZ

```
rosrun B1_viz view_robot.launch
```

Further information is available on [B1 Docs](https://www.docs.quadruped.de/projects/B1/html/index.html)
