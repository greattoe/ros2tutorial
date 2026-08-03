## rclpy_tutorial/ turtlebot3/teleop_key

##  

[튜토리얼 목록](../README.md) 

**튜토리얼 레벨 :**  초급

**빌드 환경 :**  colcon **/** Ubuntu 22.04 **/** Humble

##### 1. 사전 조사. 

발행할 토픽명과 형식을 알아내야 한다. 이를 위해 터틀봇3를 구동하고 토픽 목록을 살펴봐야 하지만 앞서 `turtlebot3_gazebo`패키지를 설치, 구동했으므로 `turtlebot3_gazebo`를 대신 구동하자.

```bash
ros2 launch turtlebot3_gazebo turtlebot3_empty_world.launch.py
```



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

```bash
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

실험 결과 `w`키 입력시 마다 `linear.x`값이 `0.01`씩 최대 `0.22`까지 증가하고, `x`키 입력시 `0.01`씩 최소 `-0.22`까지 감소하였다. 또 `a`키 입력시 마다 `angular.z`값이 `0.1`씩 최대 `2.84`까지 증가하고, `d`키 입력시 `0.1`씩 최소 `-2.84`까지 감소함. 이정보 외에 필요한 남은 정보는 `/cmd_vel`토픽의 메세지 형식 정보가 필요하다. `ros2 topic type`명령으로 알아보면 다음에서 보여지듯이 `geometry_msgs/msg/Twist`형식임을 알 수 있다. 

```bash
ros2 topic type /cmd_vel 
geometry_msgs/msg/Twist
```

이제 터틀봇3를 위한 키보드 원격 조종 노드를  작성할 모든 정보가 갖춰 졌다.

`tb3_pkg` 패키지 생성을 위해 `~/robot_ws/src`로 작업경로를 변경한다. 

```bash
cd ~/robot_ws/src
```

`ros2 pkg create`명령으로 ROS 노드 패키지 `tb3_pkg`를 생성한다. 

```bash
ros pkg create tb3_pkg --build-type ament_python
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

`__init__.py`파일과 같은 경로에 표준 입력장치((키보드)로부터 한 문자를 입력을 받아 반환하는  `getchar.py`를 작성한다. (이 코드는 ROS2 노드는 아니다. 키보드 입력을 받기위한 일종의 사용자 정의 라이브러리 이다.)

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

앞서 작성한 `getchar.py`와 같은 경로(`~/robot_ws/src/tb3_pkg/tb3_pkg`)에 키보드로 터틀봇3를 제어하는 `remote_tb3.py` 노드를 작성한다. 

```
gedit  remote_tb3.py &
```

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from tb3_pkg.get_char import GetChar


MAX_LIN_SPD  =  0.22
MIN_LIN_SPD  = -0.22
MAX_ANG_SPD  =  2.84
MIN_ANG_SPD  = -2.84
LIN_SPD_STEP =  0.01
ANG_SPD_STEP =  0.10

msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d

w/s : increase/decrease linear velocity (Burger : ~ 0.22)
a/d : increase/decrease angular velocity (Burger : ~ 2.84)

space key : force stop

CTRL-C, `Q`, `q` to quit
"""

class Remote_TB3(Node):

    def __init__(self):
        super().__init__('remote_tb3')
        
def main(args=None):
    rclpy.init(args=args)

    node = Remote_TB3()
    qos_profile = QoSProfile(depth=10)
    pub = node.create_publisher(Twist, '/cmd_vel', qos_profile)
    tw = Twist()
    kb = GetChar()
    count = 0
    
    try:
        print(msg)
        while rclpy.ok():

            if kb.chk_stdin():
                key = kb.getch()

                if key == 'w':
                    count += 1
                    if tw.linear.x + LIN_SPD_STEP <= MAX_LIN_SPD:
                        tw.linear.x += LIN_SPD_STEP
                    else:
                        tw.linear.x = MAX_LIN_SPD
                    
                elif key == 's':
                    count += 1
                    if tw.linear.x - LIN_SPD_STEP >= MIN_LIN_SPD:
                        tw.linear.x -= LIN_SPD_STEP
                    else:
                        tw.linear.x = MIN_LIN_SPD   
                         
                elif key == 'a':
                    count += 1
                    if tw.angular.z + ANG_SPD_STEP <= MAX_ANG_SPD:
                        tw.angular.z += ANG_SPD_STEP
                    else:
                        tw.angular.z = MAX_ANG_SPD
                    
                elif key == 'd':
                    count += 1
                    if tw.angular.z - ANG_SPD_STEP >= MIN_ANG_SPD:
                        tw.angular.z -= ANG_SPD_STEP
                    else:
                        tw.angular.z = MIN_ANG_SPD  
                            
                elif key == ' ' or key == 'x':
                    count += 1
                    tw.linear.x = tw.angular.z = 0.0
                
                elif key == 'q' or key == 'Q':
                    break
                                
                else:
                    pass
                pub.publish(tw)
                print("linear speed = %s, angular speed = %s." %(round(tw.linear.x,3), round(tw.angular.z,3) ))
                
                if count > 15:
                    print(msg); count = 0
        
        tw.linear.x = tw.angular.z = 0.0; pub.publish(tw)
        
    except KeyboardInterrupt:
        tw.linear.x = tw.angular.z = 0.0; pub.publish(tw)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
            

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
        ],
    },
)
```

`entry_points` 필드의 `console_scripts'` 항목에 다음 내용을 추가한다.



```python
  'remote_turtle   = tb3_pkg.remote_tb3:main',
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
                'remote_tb3   = tb3_pkg.remote_tb3:main',
        ],
    },
)
```



패키지 빌드를 위해 작업 경로를 `~/robot_ws`로 변경한다.

```
cd ~/robot_ws
```

빌드

```bash
colcon build --symlink-install --packages -select tb3_pkg
```

새로 빌드한 패키지 정보 반영을 위해 다음 명령을 실행한다.

```bash
source ~/robot_ws//instal/local_setup.bash
```

`remote_tb3.py` 노드를 구동하여 	`gazebo`에 시뮬레이션된 터틀봇3가 조종되는 가를 확인한다. 

```bash
ros2 run tb3_pkg remote_tb3 

Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d

w/s : increase/decrease linear velocity (Burger : ~ 0.22)
a/d : increase/decrease angular velocity (Burger : ~ 2.84)

space key : force stop

CTRL-C, `Q`, `q` to quit

linear speed = 0.01, angular speed = 0.0.
linear speed = 0.02, angular speed = 0.0.
linear speed = 0.02, angular speed = -0.1.
linear speed = 0.02, angular speed = -0.2.
linear speed = 0.0, angular speed = 0.0.
```







[튜토리얼 목록](../README.md) 







