## turtlebot3_tutorial/ /2_sub_odom

##  

[튜토리얼 목록](../README.md) 

**튜토리얼 레벨 :**  초급

**빌드 환경 :**  colcon **/** Ubuntu 20.04 **/** Foxy



`tb3_pkg`패키지에 `odometry` 토픽 `/odom` 토픽구독노드 `sub_odom.py` 노드를 추가해보자.

##### 1. 사전 조사. 

구독할 토픽명과 형식을 알아내야 한다. 이를 위해  `ssh` 로 터틀봇3(라즈베리파이) 에 연결한다.

```
ssh pi@10.42.0.xxx
```

ssh 연결 후, `urtlebot3_bringup robot.launch.py`를 구동한다.  

```
ros2 launch turtlebot3_bringup robot.launch.py
```

구독할 토픽명을 알아내기 위해 ` ros2 topic list`를 실행한다. 

```bash
ros2 topic list 
/battery_state
/cmd_vel
/imu
/joint_states
/magnetic_field
/odom #<------------
/parameter_events
/robot_description
/rosout
/scan
/sensor_state
/tf
/tf_static
```

 `ros2 topic list`를실행 결과에서 `/odom` 토픽을 찾을 수 있다.`./odom` 토픽의 형식을 알아내기위해 `ros2 topic type /odom` 을 실행한다. 

```
os2 topic type /odom
nav_msgs/msg/Odometry
```

`/odom`토픽의 내용, 형태를 알아 보기 위해  `ros2 topic echo /odom`명령을 실행한다.

```
ros2 topic echo /odom
header:
  stamp:
    sec: 1682546048
    nanosec: 972965940
  frame_id: odom
child_frame_id: base_footprint
pose:
  pose:
    position:
      x: 0.14234733310528327
      y: -4.379413647468286e+62
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.03743756185231476
      w: 0.9992989687588766
  covariance:
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
twist:
  twist:
    linear:
      x: -0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 1.2398843821721215e-16
  covariance:
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
```





이제 `cmd_vel` 토픽의 형식을 알아내기 위해 다음 명령을 실행한다. 

```
ros2 topic type /cmd_vel 
geometry_msgs/msg/Twist
```

`tb3_pkg` 패키지의 노드로 추가해 주기 위해  생성을 위해 `~/robot_ws/src/tb3_pkg/tb3_pkg/script`로 작업경로를 변경한다. 

```
cd ~/robot_ws/src/tb3_pkg/tb3_pkg/script
```

터틀봇3의 `odometry`토픽 구독 노드`sub_odom.py`를 작성한다. 

```
gedit sub_odom.py
```



```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
#from math import radians, degrees, pi

class SubOdom(Node):

    def __init__(self):    
        super().__init__('sub_tb3_odom')
        qos_profile = QoSProfile(depth=10)
        
        self.sub = self.create_subscription(
            Odometry,       # topic type
            'odom',         # topic name
            self.get_odom, # callback function
            qos_profile)        
        self.odom = Odometry()
    def get_odom(self, msg):
        self.odom = msg
        self.get_logger().info('x = "%s", y = "%s"' %(self.odom.pose.pose.position.x, self.odom.pose.pose.position.y))
        
        


def main(args=None):
    rclpy.init(args=args)
    node = SubOdom()
    
    try:
        rclpy.spin(node)
                
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
                'sub_odom   = tb3_pkg.script.sub_odom:main',
        ],
    },
)
```

`entry_points` 필드의 `console_scripts'` 항목에 다음 내용을 추가한다.



```python
                  'sub_odom   = tb3_pkg.script.sub_odom:main',
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

`sub_odom` 노드를 구동한다.

```
ros2 run tb3_pkgsub_odom
[INFO]: x = "0.06519161607687579", y = "-4.379413647468286e+62"
[INFO]: x = "0.06519161607687579", y = "-4.379413647468286e+62"
[INFO]: x = "0.06519161607687579", y = "-4.379413647468286e+62"
[INFO]: x = "0.06519161607687579", y = "-4.379413647468286e+62"
[INFO]: x = "0.06519161607687579", y = "-4.379413647468286e+62"
[INFO]: x = "0.06519161607687579", y = "-4.379413647468286e+62"
[INFO]: x = "0.06519161607687579", y = "-4.379413647468286e+62"
[INFO]: x = "0.06519161607687579", y = "-4.379413647468286e+62"
[INFO]: x = "0.06519161607687579", y = "-4.379413647468286e+62"
[INFO]: x = "0.0651666519763365", y = "-4.379413647468286e+62"
[INFO]: x = "0.06519161609245708", y = "-4.379413647468286e+62"
[INFO]: x = "0.06519161609245708", y = "-4.379413647468286e+62"
[INFO]: x = "0.06519161609245708", y = "-4.379413647468286e+62"
[INFO]: x = "0.06519161609245708", y = "-4.379413647468286e+62"
```











[튜토리얼 목록](../README.md) 







