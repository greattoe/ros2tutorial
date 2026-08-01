# ROS 2 (Humble Hawksbill) 설치 및 개발환경 설정

참조: <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>

### ROS2 설치(바이너리 패키지 설치)

**선행 작업: Ubuntu 22.04(Jammy Jellyfish) LTS 설치**

ROS Humble Hawksbill 바이너리 패키지는 우분투 버전 중에서는 Ubuntu 22.04(Jammy Jellyfish) 만을 지원한다. 이 후 내용은 이미 Ubuntu 22.04 LTS 버전이 설치된 상태를 전제로 한다.



#### 소스 설정

##### 시스템에 ROS 2 apt 저장소 추가

```bash
sudo apt install software-properties-common
```

```bash
sudo add-apt-repository universe
```



##### `ros2-apt-source` 패키지 설치

`ros2-apt-source` 패키지를 설치하면 시스템에 맞는 ROS 2 저장소가 구성됩니다. 이 패키지의 새 버전이 ROS 저장소에 릴리스되면 저장소 구성이 자동 업데이트된다.

```bash
sudo apt update && sudo apt install curl -y
```

```bash
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
```

```bash
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
```

```bash
sudo dpkg -i /tmp/ros2-apt-source.deb
```



#### 2. ROS2 설치

```bash
sudo apt update
```

```bash
sudo apt upgrade
```

```bash
sudo apt install ros-humble-desktop
```



##### ROS 개발 도구(ROS 패키지를 빌드하기 위한 컴파일러 및 기타 도구) 설치

```bash
sudo apt install ros-dev-tools
```



#### ROS2  설치 확인ROS설치 시 설치된  데모 노드 실행

 `talker` 퍼블리셔 노드와 `listener` 서브스크라이버 노드를 구동하여ROS2 패키지 설치를 확인해 보자.

이를 위해우선 ROS 2 환경 설정을 위한 설정 스크립트를 소싱한다.

```bash
source /opt/ros/humble/setup.bash
```



`talker` 퍼블리셔 노드 실행

```bash
ros2 run demo_nodes_cpp talker
```

```bash
ros2 run demo_nodes_cpp talker
[INFO] [1785449778.587753049] [talker]: Publishing: 'Hello World: 1'
[INFO] [1785449779.587720274] [talker]: Publishing: 'Hello World: 2'
[INFO] [1785449780.587751589] [talker]: Publishing: 'Hello World: 3'
[INFO] [1785449781.587699612] [talker]: Publishing: 'Hello World: 4'
[INFO] [1785449782.587788712] [talker]: Publishing: 'Hello World: 5'
[INFO] [1785449783.587759518] [talker]: Publishing: 'Hello World: 6'
[INFO] [1785449784.587800709] [talker]: Publishing: 'Hello World: 7'
[INFO] [1785449785.587734419] [talker]: Publishing: 'Hello World: 8'
```



다른 터미널 창을 열고 역시 ROS 2 환경 설정을 위한 설정 스크립트를 소싱한 후, 

```bash
source /opt/ros/humble/setup.bash
```

```listener``` 서브스크라이버 노드를 구동하여 동작을 확인한다.

```bash
ros2 run demo_nodes_py listener
```

```bash
ros2 run demo_nodes_py listener
[INFO] [1785449779.598723767] [listener]: I heard: [Hello World: 2]
[INFO] [1785449780.589404670] [listener]: I heard: [Hello World: 3]
[INFO] [1785449781.589335976] [listener]: I heard: [Hello World: 4]
[INFO] [1785449782.589959668] [listener]: I heard: [Hello World: 5]
[INFO] [1785449783.589462900] [listener]: I heard: [Hello World: 6]
[INFO] [1785449784.590817959] [listener]: I heard: [Hello World: 7]
[INFO] [1785449785.590165277] [listener]: I heard: [Hello World: 8]
[INFO] [1785449786.589604571] [listener]: I heard: [Hello World: 9]
```

여기까지 ROS 설치가 정상적으로 이루어진 것을 확인했다.





##### ROS2 개발 툴 설치

ROS2 로봇 프로그래밍에 필요한 소프트웨어 들을 설치한다.


```bash
sudo apt update && sudo apt install -y \
build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget
```




```bash
python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest
```




```bash
sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev \
  libcunit1-dev
```





##### ROS2 삭제

ROS2 Humble 운영 중 모종의 이유로 재설치할 필요가 생긴다면 다음 명령으로 삭제할 수 있다.

```bash
sudo apt remove '~nros-humble-*' && sudo apt autoremove
```

앞서 ROS2 설치 시 ROS2 저장소 구성을 위해 설치했던`ros2-apt-source` 패키지도 삭제하려면 다음 명령을 실행한다.

```bash
sudo apt remove '~nros-humble-*' && sudo apt autoremove
```





#### ROS2 개발환경 설정

##### 워크스페이스 폴더 생성

```bash
mkdir -p ~/robot_ws/src
```

워크스페이스 폴더의 내용 확인

```bash
ls ~/robot_ws
src

```

##### 빌드 테스트

빌드를 위해 워크스페이스로 경로 변경

```bash
cd ~/robot_ws
```



ROS 2 환경 설정을 위한 설정 스크립트 소싱

```bash
source /opt/ros/humble/setup.bash
```

빌드

```bash
colcon build --symlink-install
```



빌드 후, 워크스페이스 내용 확인

```bash
ls
build  install  log  src
```



빌드 후, `build` , `install` , `log` 폴더가 추가된 것을 확인할 수 있다. 



Run Commands 설정을 위해 `~/.bashrc`파일을 편집한다.

```bash
gedit ~/.bashrc
```

`~/.bashrc`끝부분에 아래 내용 뒤에

```bash
# enable programmable completion features (you don't need to enable
# this, if it's already enabled in /etc/bash.bashrc and /etc/profile
# sources /etc/bash.bashrc).
if ! shopt -oq posix; then
  if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
  elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
  fi
fi
```

다음을 추가한다.

```bash
######## SETTINGS FOR ROS2 HUMBLE WITH TURTLEBOT3 begin #################

source /opt/ros/humble/setup.bash
source ~/robot_ws/install/local_setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source /usr/share/colcon_cd/function/colcon_cd.sh

alias sb='source ~/.bashrc'
alias python='python3'
alias cls='clear'
alias sl='source ~/robot_ws/install/local_setup.bash'
alias cw='cd ~/robot_ws'
alias cs='cd ~/robot_ws/src'
alias ccd='colcon_cd'
alias cb='cd ~/robot_ws && colcon build --symlink-install && source ~/.bashrc'
alias cbp='cd ~/robot_ws && colcon build --symlink-install --packages-select && source ~/.bashrc'
alias circle='ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 2.0} }"'
alias stop='ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0} }"'

export _colcon_cd_root=~/robot_ws
export ROS_DOMAIN_ID=109
#export ROS_NAMESPACE=robot1
export TURTLEBOT3_MODEL=burger

######## SETTINGS FOR ROS2 HUMBLE WITH TURTLEBOT3 end ##################
```



다음은 편집 완료된 `~/.bashrc`의 전체 내용이다.

```bash
# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# If not running interactively, don't do anything
case $- in
    *i*) ;;
      *) return;;
esac

# don't put duplicate lines or lines starting with space in the history.
# See bash(1) for more options
HISTCONTROL=ignoreboth

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=1000
HISTFILESIZE=2000

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# If set, the pattern "**" used in a pathname expansion context will
# match all files and zero or more directories and subdirectories.
#shopt -s globstar

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "${debian_chroot:-}" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# set a fancy prompt (non-color, unless we know we "want" color)
case "$TERM" in
    xterm-color|*-256color) color_prompt=yes;;
esac

# uncomment for a colored prompt, if the terminal has the capability; turned
# off by default to not distract the user: the focus in a terminal window
# should be on the output of commands, not on the prompt
#force_color_prompt=yes

if [ -n "$force_color_prompt" ]; then
    if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
	# We have color support; assume it's compliant with Ecma-48
	# (ISO/IEC-6429). (Lack of such support is extremely rare, and such
	# a case would tend to support setf rather than setaf.)
	color_prompt=yes
    else
	color_prompt=
    fi
fi

if [ "$color_prompt" = yes ]; then
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
fi
unset color_prompt force_color_prompt

# If this is an xterm set the title to user@host:dir
case "$TERM" in
xterm*|rxvt*)
    PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
    ;;
*)
    ;;
esac

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    #alias dir='dir --color=auto'
    #alias vdir='vdir --color=auto'

    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# colored GCC warnings and errors
#export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'

# some more ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

# Add an "alert" alias for long running commands.  Use like so:
#   sleep 10; alert
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'

# Alias definitions.
# You may want to put all your additions into a separate file like
# ~/.bash_aliases, instead of adding them here directly.
# See /usr/share/doc/bash-doc/examples in the bash-doc package.

if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi
# enable programmable completion features (you don't need to enable
# this, if it's already enabled in /etc/bash.bashrc and /etc/profile
# sources /etc/bash.bashrc).
if ! shopt -oq posix; then
  if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
  elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
  fi
fi

######## SETTINGS FOR ROS2 HUMBLE WITH TURTLEBOT3 begin #################

source /opt/ros/humble/setup.bash
source ~/robot_ws/install/local_setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source /usr/share/colcon_cd/function/colcon_cd.sh

alias sb='source ~/.bashrc'
alias python='python3'
alias cls='clear'
alias sl='source ~/robot_ws/install/local_setup.bash'
alias cw='cd ~/robot_ws'
alias cs='cd ~/robot_ws/src'
alias ccd='colcon_cd'
alias cb='cd ~/robot_ws && colcon build --symlink-install && source ~/.bashrc'
alias cbp='cd ~/robot_ws && colcon build --symlink-install --packages-select && source ~/.bashrc'
alias circle='ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 2.0} }"'
alias stop='ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0} }"'

export _colcon_cd_root=~/robot_ws
export ROS_DOMAIN_ID=109
#export ROS_NAMESPACE=robot1
export TURTLEBOT3_MODEL=burger

######## SETTINGS FOR ROS2 HUMBLE WITH TURTLEBOT3 end ##################

```



이제 터미널 창을 새로 열면 변경사항이 반영된다. 이미 열려 있는 터미널 창에 변경사항을 반영 하려면 다음 명령을 실행한다.

```bash
source ~/.bashrc
```





[튜토리얼 목록](../README.md) 

