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

ROS2 Humble을 위한 터틀봇3 패키지는 소스코드로 배포된다.

**의존성 설치**

```bash
sudo apt install python3-argcomplete python3-colcon-common-extensions libboost-system-dev build-essential
```

```bash
sudo apt install ros-humble-hls-lfcd-lds-driver
```

```bash
sudo apt install ros-humble-turtlebot3-msgs
```

```bash
sudo apt install ros-humble-dynamixel-sdk
```

```bash
sudo apt install ros-humble-xacro
```

```bash
sudo apt install libudev-dev
```



**워크스페이스 생성 및 소스코드 복제를 위한 경로 변경**

```bash
$ mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
```



**터틀봇3 패키지 소스코드 복제**

```bash
 git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
```

**Lidar(구형) 드라이버 소스코드 복제**

```bash
git clone -b humble https://github.com/ROBOTIS-GIT/ld08_driver.git
```

**Lidar(신형) 드라이버 소스코드 복제**

```bash
git clone -b humble https://github.com/ROBOTIS-GIT/coin_d4_driver
```



터틀봇3 패키지에서 `turtlebot3_cartographer`와 `turtlebot3_navigation2` 삭제를 위한 경로 변경

```bash
cd ~/turtlebot3_ws/src/turtlebot3
```

 `turtlebot3_cartographer`와 `turtlebot3_navigation2` 삭제

```bash
rm -r turtlebot3_cartographer turtlebot3_navigation2
```



패키지 빌드를 위해 경로 변경 및 ROS 설정 반영

```bash
cd ~/turtlebot3_ws/ && source /opt/ros/humble/setup.bash
```

모든 패키지 빌드

```
colcon build --symlink-install --parallel-workers 1
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
sudo cp `ros2 pkg prefix turtlebot3_bringup`/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /etc/udev/rules.d/
```

```bash
sudo udevadm control --reload-rules
```

```bash
sudo udevadm trigger
```



```bash
echo 'export LDS_MODEL=LDS-01' >> ~/.bashrc # If you are using LDS-01
$ echo 'export LDS_MODEL=LDS-02' >> ~/.bashrc # If you are using LDS-02
$ echo 'export LDS_MODEL=LDS-03' >> ~/.bashrc # If you are using LDS-03
```





`~/.bashrc` 수정

```
nano ~/.bashrc
```

`~/.bashrc`파일 마지막 부분에서 다음 내용을 찾는다.

```
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'

# Alias definitions.
# You may want to put all your additions into a separate file like
# ~/.bash_aliases, instead of adding them here directly.
# See /usr/share/doc/bash-doc/examples in the bash-doc package.

if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
```



위 내용 이 후에 다음 내용을 추가한다.



```bash

source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/local_setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
alias python='python3'
alias cls='clear'
alias ccd='colcon_cd'
alias cw='cd ~/turtlebot3_ws'
alias cs='cd ~/turtlebot3_ws/src'
alias cb='cd ~/turtlebot3_ws && colcon build --symlink-install'
alias cbp='cd ~/turtlebot3_ws && colcon build --symlink-install --packages-select'
alias sb='source ~/.bashrc'
alias sl='source ~/turtlebot3_ws/install/local_setup.bash'

export ROS_DOMAIN_ID=30
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-03_colcon_cd_root= ~/turtlebot3_ws
```



편집 내용 저장을 위해`Ctrl`+`S`를 누른 후  `nano`편집기 종료를 위해 `Ctrl`+`X`를 입력한다.

변경된 `~/.bashrc` 반영

```bash
source ~/.bashrc
```



[완성된 이미지 다운로드](https://www.dropbox.com/scl/fi/qq7b4k0vx7oirnrjvnb2j/ROS2Humble4RPi4.tar.gz?rlkey=k0wubv4i7x1r1hr4m6rhe4pt9&st=uenpxrwo&dl=0)



```
ros2 launch turtlebot3_bringup robot.launch.py
```





[튜토리얼 목록](../README.md) 







