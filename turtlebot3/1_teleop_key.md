## turtlebot3_tutorial/ /1_teleop_key

##  

[튜토리얼 목록](../README.md) 

**튜토리얼 레벨 :**  초급

**빌드 환경 :**  colcon **/** Ubuntu 20.04 **/** Foxy



`turtlebot3`  관련 노드 작성을 위해  `tb3_pkg`패키지를 만들고 키보드로 터틀봇3를 원격 조종하는 노드 `remote_turtle.py`를 작성해보자. 

##### 1. 사전 조사. 

발행할 토픽명과 형식을 알아내야 한다. 이를 위해  `ssh` 로 터틀봇3(라즈베리파이) 에 연결한다.

```
ssh pi@10.42.0.xxx
```

ssh 연결 후, `urtlebot3_bringup robot.launch.py`를 구동한다.  

```
ros2 launch turtlebot3_bringup robot.launch.py
```

발행할 토픽명을 알아내기 위해 ` ros2 topic list`를 실행한다. 

```bash
ros2 topic list 
/battery_state
/cmd_vel #<------------
/imu
/joint_states
/magnetic_field
/odom
/parameter_events
/robot_description
/rosout
/scan
/sensor_state
/tf
/tf_static
```

 `ros2 topic list`를실행 결과를 살펴보면, 터틀봇3를 원격 조종하기위해 발행해야 하는 토픽이 `/cmd_vel`이라는 것을 추측할 수 있지만 확인을 위해 해당 토픽을 `echo` 시키고, `teleop_keyboard` 노드를 실행하여 그 내용의 변화를 관찰한다.

```
ros2 topic echo /cmd_vel
linear:
  x: 0.02
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -0.1
```

이제 `cmd_vel` 토픽의 형식을 알아내기 위해 다음 명령을 실행한다. 

```
ros2 topic type /cmd_vel 
geometry_msgs/msg/Twist
```

`tb3_pkg` 패키지 생성을 위해 `~/robot_ws/src`로 작업경로를 변경한다. 

```
cd ~//robot_ws/src
```

`rclpy`와 `geometry_msgs`에 대한 의존성을 가지는 ROS 노드 패키지 `tb3_pkg`를 생성한다. 

```bash
ros pkg create tb3_pkg --build-type ament_python --dependencies rclpy geometry_msgs
```



이제 필요한 정보는 `turtle1/cmd_vel` 토픽이 어떤 토픽인가 이다. 이를 알아내기 위해 `ros2 topic type` 명령을 이용하자. 

```bash
ros2 topic type /turtle1/cmd_vel
geometry_msgs/msg/Twist
```

`~/robot_ws/src/tb3_pkg/tb3_pkg`폴더로 작업 경로 변경

```bash
cd ~/robot_ws/src/tb3_pkg/tb3_pkg
```

`ls` 명령으로 작업경로에 `__init__.py`파일의 존재를 확인한다.

```bash
ls_*.py
__init__.py
```

`__init__.py`파일과 같은 경로에 키보드로부터 한 무자를 입력을 받는  `getchar.py`를 작성한다. (이 코드는 ROS2 노드는 아니다. 키보드 입력을 받기위한 일종의 사용자 라이브러리 이다.)

```bash
gedit getchar.py &
```

```python
import os, time, sys, termios, atexit, tty
from select import select
  
# class for checking keyboard input
class Getchar:
    def __init__(self):
        # Save the terminal settings
        self.fd = sys.stdin.fileno()
        self.new_term = termios.tcgetattr(self.fd)
        self.old_term = termios.tcgetattr(self.fd)
  
        # New terminal setting unbuffered
        self.new_term[3] = (self.new_term[3] & ~termios.ICANON & ~termios.ECHO)
        termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.new_term)
  
        # Support normal-terminal reset at exit
        atexit.register(self.set_normal_term)      
      
    def set_normal_term(self):
        termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.old_term)
  
    def getch(self):        # get 1 byte from stdin
        """ Returns a keyboard character after getch() has been called """
        return sys.stdin.read(1)
  
    def chk_stdin(self):    # check keyboard input
        """ Returns True if keyboard character was hit, False otherwise. """
        dr, dw, de = select([sys.stdin], [], [], 0)
        return dr
```





##### 터틀봇3 키보드 원격 조종 노드 작성

`터틀봇3와 관련된 몇가지 코드를 작성할 ``script` 폴더를 만들고  경로를 해당 폴더로 변경한다. 

```
mkdir script && cd script
```

키보드로 `turtlsim`노드의 거북이를 제어하는 `remote_tb3` 노드를 작성한다. 

```
gedit  remote_tb3.py &
```

```python
import rclpy, sys
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Twist
from turtle_pkg.getchar import Getchar
MAX_LIN_SPD = 0.22
MIN_LIN_SPD = -0.22
MAX_ANG_SPD = 2.84
MIN_ANG_SPD = -2.84
lin_spd = 0.0
ang_spd = 0.0
LIN_STEP = 0.01
ANG_STEP = 0.04
msg = """    forward
              +---+
              | w |
          +---+---+---+
turn left | a | s | d | turn right
          +---+---+---+
              | x | 
              +---+   
             backward
             
### space or 's' for stop\n
"""

class RemoteTb3(Node):

    def __init__(self):
        self.cnt_sec = 0
        super().__init__('remote_tb3')
        qos_profile = QoSProfile(depth=10)
        #self.pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.timer    = self.create_timer(1, self.count_sec)

    def count_sec(self):
        self.cnt_sec = self.cnt_sec + 1
        #print(self.cnt_sec)


def main(args=None):
    global lin_spd, ang_spd, MAX_LIN_SPD, MIN_LIN_SPD, MAX_ANG_SPD, MIN_ANG_SPD
    rclpy.init(args=args)
    node= RemoteTb3()
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    tw = Twist()
    kb = Getchar()
    key = ' '
    count = 0
    print(msg)
    try:
            while rclpy.ok():
                key = kb.getch()
                if      key == 'w':
                        if lin_spd + LIN_STEP <= MAX_LIN_SPD:
                            lin_spd = lin_spd + LIN_STEP
                        else:
                            lin_spd = MAX_LIN_SPD
                elif key == 'x':
                        if lin_spd - LIN_STEP >= MIN_LIN_SPD:
                            lin_spd = lin_spd - LIN_STEP
                        else:
                            lin_spd = MIN_LIN_SPD
                elif      key == 'a':
                        if ang_spd + ANG_STEP <= MAX_ANG_SPD:
                            ang_spd = ang_spd + ANG_STEP
                        else:
                            ang_spd = MAX_ANG_SPD
                elif key == 'd':
                        if ang_spd - ANG_STEP >= MIN_ANG_SPD:
                            ang_spd = ang_spd - ANG_STEP
                        else:
                            ang_spd = MIN_ANG_SPD
                elif key == ' ':
                    lin_spd = ang_spd = 0.0
                elif key == 's':
                    lin_spd = ang_spd = 0.0
                
                
                tw.linear.x = lin_spd
                tw.angular.z = ang_spd
                pub.publish(tw)
                node.get_logger().info('linear.x = "%s", angular.z = "%s"' %(lin_spd, ang_spd))
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt(SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
```



`setup.py` 파일 편집을 위해 경로를 `~/robot_ws/src/tb3_pkg`로 변경한다. 

```
cd ~/robot_ws/src/tb3_pkg
```



`setup.py` 파일 편집

```
gedit setup.py &
```



```python
from setuptools import find_packages
from setuptools import setup

package_name = 'tb3_pkg'

setup(
    name=package_name,  
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gnd0',
    maintainer_email='greattoe@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'remote_turtle   = tb3_pkg.script.remote_turtle:main',
        ],
    },
)
```

`entry_points` 필드의 `console_scripts'` 항목에 다음 내용을 추가한다.



```python
  'remote_turtle   = tb3_pkg.script.remote_turtle:main',
```



패키지 빌드를 위해 작업 경로를 `~/robot_ws`로 변경한다.

```
cd ~/robot_ws
```

빌드

```
colcon build --symlink-install
```

새로 빌드한 패키지 정보 반영을 위해 다음 명령을 실행한다.

```
. instal/local_setup.bash
```

`remote_turtle` 노드를 구동하려 `turtlesim` 노드의 거북이가 조종되는 가를 확인한다. 

```
ros2 run tb3_pkg remote_tb3 
           forward
              +---+
              | w |
          +---+---+---+
turn left | a | s | d | turn left
          +---+---+---+
             backward
### space for stop


[INFO]: linear.x = "0.01", angular.z = "0.0"
[INFO]: linear.x = "0.02", angular.z = "0.0"
[INFO]: linear.x = "0.03", angular.z = "0.0"
[INFO]: linear.x = "0.03", angular.z = "-0.04"
[INFO]: linear.x = "0.03", angular.z = "-0.08"
[INFO]: linear.x = "0.03", angular.z = "-0.12"
```











[튜토리얼 목록](../README.md) 







