### 터틀봇3 운영을 위한 ROS2 (Foxy Fitzroy) 재설치

참조1: <https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html>

참조2:<https://cafe.naver.com/openrt/25288>

**선행 작업: Ubuntu 20.04(Focal Fossa) LTS 설치**

ROS Foxy Fitzroy 바이너리 패키지는 우분투 버전 중에서는 Ubuntu 20.04(Focal Fossa) 만을 지원한다. 이 후 내용은 이미 Ubuntu 20.04 LTS 버전이 설치된 상태를 전제로 한다.



#### ROS2 삭제

```
sudo apt remove ~nros-foxy-* && sudo apt autoremove
```



#### curl 설치

```bash
sudo apt update && sudo apt install curl -y
```



#### ROS2 GPG Key 추가

```bash

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```



#### `sources.list`에 ROS2 Repository 추가

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```



#### 추가된 ROS2 Repository 시스템에 반영

```bash
sudo apt update
```



#### ROS2 데비안 패키지(GUI Tool 포함) 설치

```
sudo apt install ros-foxy-desktop ros-foxy-rmw-fastrtps* ros-foxy-rmw-cyclonedds*
```



#### Turtlebot3를 위한 ROS2 의존성 패키지 설치

##### 시뮬레이터(Gazebo11) 설치

```
sudo apt-get install ros-foxy-gazebo-*
```



##### SLAM 패키지(Cartographer) 설치

```
 sudo apt install ros-foxy-cartographer*
```

```
sudo apt install ros-foxy-cartographer-ros
```







##### 네비게이션 패키지(-Navigation2) 설치

```
sudo apt install ros-foxy-navigation2
```

```
sudo apt install ros-foxy-nav2-bringup
```



#### Turtlebot3 패키지 설치

```
sudo apt install ros-foxy-dynamixel-sdk
```

```
sudo apt install ros-foxy-turtlebot3-msgs
```

```
sudo apt install ros-foxy-turtlebot3
```







##### ROS2 패키지 설치 확인

 `talker` 퍼블리셔 노드와 `listener` 서브스크라이버 노드를 구동하여ROS2 패키지 설치를 확인해 보자.

이를 위해우선 ROS 2 환경 설정을 위한 설정 스크립트를 소싱한다.

```
source /opt/ros/foxy/setup.bash
```

`talker` 퍼블리셔 노드 실행

```
ros2 run demo_nodes_cpp talker
```

```
ros2 run demo_nodes_cpp talker
[INFO] [1612912263.574031946] [talker]: Publishing: 'Hello World: 1'
[INFO] [1612912264.574010597] [talker]: Publishing: 'Hello World: 2'
[INFO] [1612912265.574381893] [talker]: Publishing: 'Hello World: 3'
[INFO] [1612912266.574508130] [talker]: Publishing: 'Hello World: 4'
[INFO] [1612912267.574615200] [talker]: Publishing: 'Hello World: 5'
[INFO] [1612912268.574767202] [talker]: Publishing: 'Hello World: 6'
[INFO] [1612912269.574953419] [talker]: Publishing: 'Hello World: 7'
...
```

다른 터미널 창을 열고 역시 ROS 2 환경 설정을 위한 설정 스크립트를 소싱한 후, 

```
source /opt/ros/foxy/setup.bash
```

```listener``` 서브스크라이버 노드를 구동하여 동작을 확인한다.

```
ros2 run demo_nodes_py listener
```

```
ros2 run demo_nodes_py listener
[INFO] [1612912265.593335793] [listener]: I heard: [Hello World: 3]
[INFO] [1612912266.576514520] [listener]: I heard: [Hello World: 4]
[INFO] [1612912267.576780341] [listener]: I heard: [Hello World: 5]
[INFO] [1612912268.576769156] [listener]: I heard: [Hello World: 6]
[INFO] [1612912269.577142775] [listener]: I heard: [Hello World: 7]
```



##### ROS2 개발 툴 설치

ROS2 로봇 프로그래밍에 필요한 소프트웨어 들을 설치한다.


```
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




```
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




```
sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev \
  libcunit1-dev
```



#### ROS2 개발환경 설정

##### 워크스페이스 폴더 생성

```
mkdir -p ~/robot_ws/src
```

워크스페이스 폴더의 내용 확인

```
ls ~/robot_ws
src

```

##### 빌드 테스트

빌드를 위해 워크스페이스로 경로 변경

```
cd ~/robot_ws
```



ROS 2 환경 설정을 위한 설정 스크립트 소싱

```
source /opt/ros/foxy/setup.bash
```

빌드

```
colcon build --symlink-install                     
Summary: 0 packages finished [0.17s]
```



빌드 후, 워크스페이스 내용 확인

```
ls
build  install  log  src
```



빌드 후, `build` , `install` , `log` 폴더가 추가된 것을 확인할 수 있다. 



Run Commands 설정을 위해 `~/.bashrc`파일을 편집한다.

```
gedit ~/.bashrc
```

`~/.bashrc`끝부분에 `fi`가 적힌 행이 2줄 연속으로 나타나는 부분을 찾는다. (116, 117행 즈음)

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
fi # <<============
fi # <<============
```

`fi`가 적힌 행이 2줄 뒤에 아래 내용을 추가한다. 

(`~/.bashrc` 파일에 추가할 내용)

```bash
source /opt/ros/foxy/setup.bash
source ~/robot_ws/install/local_setup.bash

source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source /usr/share/vcstool-completion/vcs.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=~/robot_ws

export ROS_DOMAIN_ID=7
export ROS_NAMESPACE=robot1

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# export RMW_IMPLEMENTATION=rmw_connext_cpp
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# export RMW_IMPLEMENTATION=rmw_gurumdds_cpp

# export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})'
export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}]: {message}'
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_LOGGING_USE_STDOUT=0
export RCUTILS_LOGGING_BUFFERED_STREAM=1

alias cw='cd ~/robot_ws'
alias cs='cd ~/robot_ws/src'
alias ccd='colcon_cd'

alias cb='cd ~/robot_ws && colcon build --symlink-install'
alias cbs='colcon build --symlink-install'
alias cbp='colcon build --symlink-install --packages-select'
alias cbu='colcon build --symlink-install --packages-up-to'
alias ct='colcon test'
alias ctp='colcon test --packages-select'
alias ctr='colcon test-result'

alias rt='ros2 topic list'
alias re='ros2 topic echo'
alias rn='ros2 node list'

alias killgazebo='killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient'

alias af='ament_flake8'
alias ac='ament_cpplint'

alias testpub='ros2 run demo_nodes_cpp talker'
alias testsub='ros2 run demo_nodes_cpp listener'
alias testpubimg='ros2 run image_tools cam2image'
alias testsubimg='ros2 run image_tools showimage'
```



**`.bashrc` 파일 example**

다음은 그렇게 편집한 `~/.bashrc`파일 예시이다. 

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

source /opt/ros/foxy/setup.bash
source ~/robot_ws/install/local_setup.bash
# source ~/turtlebot3_ws/install/local_setup.bash

export TURTLEBOT3_MODEL=burger
#export TURTLEBOT3_MODEL=${TB3_MODEL}

source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source /usr/share/vcstool-completion/vcs.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=~/robot_ws

export ROS_DOMAIN_ID=19
#export ROS_NAMESPACE=robot1

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# export RMW_IMPLEMENTATION=rmw_connext_cpp
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# export RMW_IMPLEMENTATION=rmw_gurumdds_cpp

# export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})'
export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}]: {message}'
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_LOGGING_USE_STDOUT=0
export RCUTILS_LOGGING_BUFFERED_STREAM=1

alias cw='cd ~/robot_ws'
alias cs='cd ~/robot_ws/src'
alias si='source ~/robot_ws/install/setup.bash'
alias sb='source ~/.bashrc'
alias cls='clear'


alias ccd='colcon_cd'
alias cb='cd ~/robot_ws && colcon build --symlink-install && source ~/.bashrc'
alias cbs='colcon build --symlink-install'
alias cbp='colcon build --symlink-install --packages-select'
alias cbu='colcon build --symlink-install --packages-up-to'
alias ct='colcon test'
alias ctp='colcon test --packages-select'
alias ctr='colcon test-result'

alias rt='ros2 topic list'
alias re='ros2 topic echo'
alias rn='ros2 node list'

alias killgazebo='killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient'

alias af='ament_flake8'
alias ac='ament_cpplint'

alias testpub='ros2 run demo_nodes_cpp talker'
alias testsub='ros2 run demo_nodes_cpp listener'
alias testpubimg='ros2 run image_tools cam2image'
alias testsubimg='ros2 run image_tools showimage'

alias nb='nano ~/.bashrc'
alias sb='source ~/.bashrc'
alias gs='git status'
alias gp='git pull'
alias sb='source ~/.bashrc'

alias circle_turtle='ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 2.0} }"'
alias stop_turtle='ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0} }"'

```



이제 터미널 창을 새로 열면 변경사항이 반영된다. 이미 열려 있는 터미널 창에 변경사항을 반영 하려면 다음 명령을 실행한다.

```bash
source ~/.bashrc
```





[튜토리얼 목록](./README.md) 

