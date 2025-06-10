## turtlebot3_tutorial/ /Simulation

##  

[튜토리얼 목록](../README.md) 

**튜토리얼 레벨 :**  초급

**빌드 환경 :**  colcon **/** Ubuntu 20.04 **/** Foxy



우분투 20.04 및 ROS Foxy가 설치되고, Turtlebot3를 운영하기 위해 Remote PC Setup이 완료된 상태에서 Turtlebot3 시뮬레이션을 통한 SLAM Navigation 실습을 해보자.

### 1. Turtlebot3 Gazebo 시뮬레이션 패키지 설치 및 구동

Turtlebot3 시뮬레이션 패키지는 ROBOTIS  [ROBOTIS 깃허브](https://github.com/ROBOTIS-GIT) 에서 소스코드 형태로 제공되는 것을 다운로드받아 빌드해야한다. 

Turtlebot3 시뮬레이션 패키지 빌드를 위한 워크스페이스 생성

```bash
mkdir -p ~/turtlebot3_ws/src
```



Turtlebot3 시뮬레이션 패키지 소스코드를 클론하기 위한 작업경로 변경

```bash
cd ~/turtlebot3_ws/src
```



Turtlebot3 시뮬레이션 패키지 소스코드 복제(복사 또는 다운로드)

```bash
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```



복제한 Turtlebot3 시뮬레이션 패키지코드 빌드

```bash
  cd ~/turtlebot3_ws && colcon build --symlink-install
```



다음 내용을 `~/.bashrc`파일에 추가 후, 반영한다.)

```bash
source ~/turtlebot3_ws/install/local_setup.bash
export TURTLEBOT3_MODEL=burger
```



Turtlebot3 시뮬레이션 패키지 구동

Gazebo시뮬레이션을 구동할 때는 `World`라고하는 가상의 시뮬레이션 환경을 `Loading`하여 구동하는데, Turtlebot3는 3개(`empty_world`, `turtlebot3_world`, `turtlebot3_house`)의 시뮬레이션 환경이 제공되며, 각각의 환경(World)에 대한 `launch`파일이 제공되므로 해당 `launch`파일을 구동하여 시뮬레이션을 수행할 수 있다.

**Empty World**

```bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

![](../img/turtlebot3_empty_world.png)



**TurtleBot3 World**

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

![](../img/turtlebot3_turtlebot3_world.png)



**turtlebot3_house**

![](../img/turtlebot3_house.png)



### 2. Gazebo 시뮬레이션을 이용한 터틀봇3 SLAM 실습

터틀봇3 Gazebo 시뮬레이션 패키지를 Turtlebot3_World 시뮬레이션 환경에서 구동 후,터틀봇3 SLAM 패키지를 구동하여 맵을 작성하는 실습을 해보자.

**2.1 터틀봇3 Gazebo 시뮬레이션 패키지 구동(Turtlebot3_World)**

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

![](https://github.com/greattoe/ros2_tutorial/blob/master/img/turtlebot3_gazebo_turtlebot3_world.png)



**2.2 터틀봇3 Cartographer 슬램(SLAM) 패키지 구동**

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```



![](https://github.com/greattoe/ros2_tutorial/blob/master/img/turtlebot3_cartographer.png)



**2.2 터틀봇3 원격 조종 노드(Teleoperation Node) 실행**

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

```bash
Control Your TurtleBot3!
 ---------------------------
 Moving around:
        w
   a    s    d
        x

 w/x : increase/decrease linear velocity
 a/d : increase/decrease angular velocity
 space key, s : force stop

 CTRL-C to quit
```



원격조종노드를 구동, 시뮬레이션된 터틀봇3를 조종하여, rviz화면의 어두운 부분을 구석 구석 밝혀 준다. 

![](https://github.com/greattoe/ros2_tutorial/blob/master/img/turtlebot3_cartographer2.png)

**2.3 작성한 맵 저장하기**

![](https://github.com/greattoe/ros2_tutorial/blob/master/img/virtual_slam.png)

맵을 작성할 지형 구석구석 충분히 스캔하여 rviz화면의 모든 영역이 밝은색으로 표시되었으면 다음 명령을 실행하여 현재 상태를 지도로 저장한다.

```
ros2 run nav2_map_server map_saver_cli -f ~/map
```



![](https://github.com/greattoe/ros2_tutorial/blob/master/img/saved_map.png)

`map.pgm`, `map.yaml`파일이 생성된 것을 확인할 수 있다.



### 3. Gazebo 시뮬레이션을 이용한 터틀봇3 Navigation 실습

Gazebo 시뮬레이션 환경에서 앞서 작성한 Turtlebot3_World map을 이용하여 터틀봇3 네비게이션 실습을 해보자.



**3.1 터틀봇3 Gazebo 시뮬레이션 패키지 구동(Turtlebot3_World)**

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```



**3.2 터틀봇3 네비게이션 패키지 구동**

```bash
 ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml
```





[튜토리얼 목록](../README.md) 







