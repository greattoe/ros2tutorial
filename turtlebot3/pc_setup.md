## turtlebot3_tutorial/ /pc_setup

##  

[튜토리얼 목록](../README.md) 

**튜토리얼 레벨 :**  초급

**빌드 환경 :**  colcon **/** Ubuntu 20.04 **/** Foxy



우분투 20.04 설치 후, ROS Foxy 설치가 완료된 상태에서 Turtlebot3를 운영하기 위해 Remote PC에서 다음 작업들을 수행한다. 

### Turtlebot3를 위한 ROS2 의존성 패키지 설치

**시뮬레이터(Gazebo11) 설치**

```bash
sudo apt-get install ros-foxy-gazebo-*
```



**SLAM 패키지(Cartographer) 설치**

```bash
 sudo apt install ros-foxy-cartographer
 sudo apt install ros-foxy-cartographer-ros
```



**네비게이션 패키지(-Navigation2) 설치**

```bash
sudo apt install ros-foxy-navigation2
 sudo apt install ros-foxy-nav2-bringup
```



### Turtlebot3 패키지 설치

```bash
sudo apt install ros-foxy-dynamixel-sdk
sudo apt install ros-foxy-turtlebot3-msgs
sudo apt install ros-foxy-turtlebot3
```














[튜토리얼 목록](../README.md) 







