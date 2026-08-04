## 터틀봇3 SBC(Raspberrypi) 설정





---

터틀봇3의  미션 컴퓨터에 해당하는 라즈베리파이 설정하기.

**출처 :**  <https://docs.robotis.com/docs/systems/turtlebot3/quick_start_guide/sbc_setup>

**OS :**  Ubuntu Server 22.04 LTS

**ROS :**  Humble Hawksbill

---

### Raspberry Pi Imager 설치

```bash
sudo apt install rpi-imager
```







#### 기타설정

**업데이트**

```
sudo apt-update
```



**업그레이드**

```
sudo apt upgrade
```



**`net-tools`설치**

```
sudo apt install net-tools
```



**`openssh-server`삭제**

```
sudo apt remove openssh-server
```





**`openssh-server`설치**

```
sudo apt install openssh-server
```



**자동 업데이트 설정 변경**

```bash
sudo nano /etc/apt/apt.conf.d/20auto-upgrades
```

```bash
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Unattended-Upgrade "0";
```



**네트워크 연결이 없을 경우 부팅 지연 방지**

```
sudo systemctl mask systemd-networkd-wait-online.service
```



**절전 및 최대 절전 모드 비활성화**

```
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
```



**시스템 재시작**

```
sudo reboot
```



#### ROS2 Humble Hawksbill  설치



**Universe 저장소 활성화**

```bash
sudo apt install software-properties-common -y
```

```bash
sudo add-apt-repository universe
```



**ROS 저장소 등록 도구 설치**

```bash
sudo apt install curl gnupg lsb-release -y
```



**ROS key 저장**

```bash
sudo curl -sSL \
https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
-o /usr/share/keyrings/ros-archive-keyring.gpg
```



**ROS 2 저장소 등록**

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```



**ROS2 저장소 등록 확인**

```bash
cat /etc/apt/sources.list.d/ros2.list
```

```bash
deb [arch=arm64 ...] http://packages.ros.org/ros2/ubuntu jammy main
```



**ROS 2 Humble 설치**

```bash
sudo apt update && sudo apt install ros-humble-ros-base -y
```



**개발 도구 설치**

```bash
sudo apt install ros-dev-tools -y
```



**데모 패키지를 설치**

```bash
sudo apt install ros-humble-demo-nodes-cpp -y
```

**데모 노드 실행**

- 첫 번째 터미널:

  ```bash
  source /opt/ros/humble/setup.bash && ros2 run demo_nodes_cpp talker
  ```


- 두 번째 터미널:

- ```bash
  source /opt/ros/humble/setup.bash && ros2 run demo_nodes_cpp listener
  ```





#### 터틀봇3 패키지 설치



**의존성 설치**

```bash
sudo apt install python3-argcomplete python3-colcon-common-extensions libboost-system-dev build-essential  ros-humble-hls-lfcd-lds-driver ros-humble-turtlebot3-msgs \
ros-humble-dynamixel-sdk apt install ros-humble-xacro libudev-dev
```

```
sudo apt install libboost-dev libboost-system-dev -y
```

```
sudo apt install libudev-dev -y
```

```
sudo apt install ros-humble-xacro -y
```



**워크스페이스 생성 및 경로 변경**

```
$ mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
```



**터틀봇3 패키지 소스코드 복제**

```
 git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
```

**Lidar(구형) 드라이버 소스코드 복제**

```
git clone -b humble https://github.com/ROBOTIS-GIT/ld08_driver.git
```

**Lidar(신형) 드라이버 소스코드 복제**

```
git clone -b humble https://github.com/ROBOTIS-GIT/coin_d4_driver
```



터틀봇3 패키지에서 `turtlebot3_cartographer`와 `turtlebot3_navigation2` 삭제를 위해 경로 변경

```
cd ~/turtlebot3_ws/src/turtlebot3
```

 `turtlebot3_cartographer`와 `turtlebot3_navigation2` 삭제

```
rm -r turtlebot3_cartographer turtlebot3_navigation2
```









```bash
$ 

$
$ git clone -b humble https://github.com/ROBOTIS-GIT/ld08_driver.git
$ 
$ cd ~/turtlebot3_ws/src/turtlebot3
$ rm -r turtlebot3_cartographer turtlebot3_navigation2
$ cd ~/turtlebot3_ws/
$ echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
$ source ~/.bashrc
$ colcon build --symlink-install --parallel-workers 1
$ echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
$ source ~/.bashrc
```



#### OpenCR용 USB 포트 설정

```bash
$ sudo cp `ros2 pkg prefix turtlebot3_bringup`/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger
```



```
echo 'export LDS_MODEL=LDS-01' >> ~/.bashrc # If you are using LDS-01
$ echo 'export LDS_MODEL=LDS-02' >> ~/.bashrc # If you are using LDS-02
$ echo 'export LDS_MODEL=LDS-03' >> ~/.bashrc # If you are using LDS-03
```









[튜토리얼 목록](../README.md) 







