### rclpy Params Tutorial3

**[Parameter의 사용 1](./parameter1.md)**은 `parameter`를 등록한 노드에서 `parameter`값을 읽거나, 변경하는 방법을 다루었고, 

**[Parameter의 사용 2](./parameter2.md)**는 `parameter`를 다른 노드에서 등록한 `parameter`값을 읽거나, 변경하는 방법을 다루었다.

다른 노드에서 등록한 `parameter`값을 읽거나, 변경하는 것이 상당히 까다롭다는 것을 알 수 있다. 이 문서에서는 약간의 편법을 이용해서 그 것을 어느정도 해결해보려 한다. 이 방법은 `parameter가 어디에서 등록되었는 가에 상관없이 값을 읽거나, 변경할 수 있지만 다음 2가지 전제 조건을 지켜야 한다.  

1. 문자열 형식의 `parameter`만 사용한다. 
2. 필요한 `parameter`는 `reg_params.py`에서 모두 등록하기로 한다.
3. 

`ros2` 패키지 `ex_param` 생성을 위해 작업 경로를 `~/robot_ws/src` 로 변경한다. 

```bash
cd ~robot_ws/src
```

다음 명령으로 `rclpy`에 의존성을 갖는 ROS2 패키지 `ex_param`을 생성한다.



```bash
ros2 pkg create ex_param --build-type ament_python --dependencies rclpy
```



작업 경로를 `~/robot_ws/src/ex_param/ex_param` 으로 변경한다.

```bash
cd ~/robot_ws/src/ex_param/ex_param
```

필요한 `parameter`들을 ROS `parameter`서버에 등록 해 주는 `reg_params.py` 를 작성한다.

```bash
gedit reg_params.py &
```



```python
import rclpy, os
from rclpy.node import Node

from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

class RegParams(Node):
    def __init__(self):
        super().__init__('reg_params')

        self.declare_parameter("move_turtle", "stop")
        '''
        self.declare_parameter("param1", ["param1's initial value"]) # decrare parametr1 you want
        self.declare_parameter("param2", ["param2's initial value"]) # decrare parametr2 you want
        self.declare_parameter("param3", ["param3's initial value"]) # decrare parametr3 you want
        '''

def main():
    rclpy.init()
    node = RegParams()   
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```



`setup.py`의 `entry_points`에 `reg_params.py`를추가한다. 



```bash
gedit ~/robot_ws/src/ex_param/setup.py &
```

```python
from setuptools import setup

package_name = 'ex_param'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
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
                'reg_params = ex_param.reg_params:main',# <============== add this line
        ],
    },
)
```



빌드를 위해 작업 경로를 `~/robot_ws`로 변경한다. 

```bash
cd ~/robot_ws
```



다음 명령을 수행하여 `ex_param` 패키지를 빌드한다. 

```bash
colcon build --symlink-install --packages-select ex_param
```



새로 빌드한 패키지( `ex_param` ) 정보 반영을 위해 다음 명령을 실행한다. 



```bash
source ./install/local_setup.bash
```



다음 명령을 수행하여 `reg_params.py`를 실행한다. 

```
ros2 run ex_param reg_params
```



다음 명령으로 `move_turtle` `parameter`가 등록되었는 지 확인한다.

```bash
ros2 param list
```

```bash
/reg_params:
  move_turtle
```

`reg_params`노드의 `move_turtle` `parameter`가 등록된 것을 알 수 있다. 



다음 명령으로 `move_turtle` `parameter`의 값을 확인해보자.

```
ros2 param get /reg_params move_turtle
```

```
String value is: stop
```



다음 명령으로 `move_turtle` `parameter`의 값을 `go`로 변경해보자.

```
ros2 param set /reg_params move_turtle go
```

```
Set parameter successful
```



`move_turtle` `parameter`의 값이 변경되었는 지 확인해보자.

```
ros2 param get /reg_params move_turtle
```

```
String value is: go
```



다시 `move_turtle` `parameter`의 값을 `stop`으로 바꾸어 두자.

```
ros2 param set /reg_params move_turtle stop
```

```
Set parameter successful
```





작업경로를 `~/robot_ws/src/ex_param/ex_param`으로 변경한다. 

```
cd ~/robot_ws/src/ex_param/ex_param
```

`parameter` `move_turtle`의 값에 따라 `turtlesim` 노드의 거북이를 제어하는 `turtle_by_param.py`작성을 위해 다음 명령을 실행 한다. 

```bash
gedit turtle_by_param.py &
```



```python
import rclpy, sys, os
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
#from rclpy.exceptions import ParameterNotDeclaredException
#from rcl_interfaces.msg import ParameterType

class ByParam(Node):
    def __init__(self):
        super().__init__('turtle_by_param')

def main():
    rclpy.init()
    node = ByParam()
    qos_profile = QoSProfile(depth=10)
    pub = node.create_publisher(Twist, '/turtle1/cmd_vel', qos_profile)
    tw = Twist()
    try:
        while rclpy.ok():
            str = os.popen("ros2 param get /reg_params move_turtle").read()
            param = str[17:].strip()
            
            if param == "go":
                tw.linear.x = 0.5; tw.angular.z = 0.25
            
            elif param == "stop":
                tw.linear.x = tw.angular.z = 0.0
            else:
                pass
            node.get_logger().info('turtle %s!' % param)
            pub.publish(tw)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt(SIGINT)')
        
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()
```



`setup.py`를 아래와 같이 편집 후, 저장한다. 



```python
from setuptools import find_packages
from setuptools import setup

package_name = 'ex_param'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
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
                'reg_params = reg_param.reg_params:main',
                'turtle_by_param = ex_param.turtle_by_param:main', # <== 추가
        ],
    },
)
```



동작 테스트를 위해선 `turtlesim`, `reg_params`, `turtle_by_param` 3개의 `node`를 실행해야한다. 

일단 `turtlesim` 노드 실행을 위해 다음 명령을 실행한다. 

```bash
ros2 run turtlesim turtlesim_node
```

`reg_params`실행을 위해 다음 명령을 실행한다. 

```
ros2 run ex_param reg_params
```

앞서 작성한 노드 `turtle_by_param` 노드 실행을 위해 다음 명령을 실행한다. 

```
ros2 run ex_param turtle_by_params
```

```bash
[INFO]: turtle stop!
[INFO]: turtle stop!
[INFO]: turtle stop!
```



새 터미널 창에서 `ros2 param list` 명령을 실행한다. 

```
ros2 param list 
/reg_params:
  move_turtle
  use_sim_time
/turtlesim:
  background_b
  background_g
  background_r
  use_sim_time
```



`ros2 param list` 명령의 실행 결과 화면에서 `reg_params` 노드는 파라미터 `move_turtle` 과 `use_sim_time` 을, `turtlesim` 노드는 파라미터 `background_b`, `background_g`, `background_r`,`use_sim_time` 을 가지고 있다는 것을 알 수 있다. 

다음 명령으로 `move_by_param` 노드의 파라미터 `go_turtle`의 값을 알아보자.

```
ros2 param get /move_by_param go_turtle 
String value is: stop
```



문자열`stop`이라는 것을 알 수 있다. 또 다른 터미널 창을 열어 다음 명령으로 파라미터 `go_turtle`의 값을 문자열 `go`로 변경해보자.

```
os2 param set /move_by_param go_turtle go
Set parameter successful
```



앞서 `turtle_by_param` 노드 실행 창의 출력이 다음과 같이 바뀐 것을 확인할 수 있다.

```
ros2 run param_tutorial turtle_by_param 
[INFO]: turtle stop!
[INFO]: turtle stop!
[INFO]: turtle stop!
[INFO]: turtle stop!
[INFO]: turtle stop!
[INFO]: turtle go!
[INFO]: turtle go!
[INFO]: turtle go!
[INFO]: turtle go!
[INFO]: turtle go!
[INFO]: turtle go!
```

동시에 `turtlesim`노드의 거북이가 움직이기 시작한 것도 확인할 수 있다.

<img src="./img/turtle_move_by_param.png">



`turtlesim_node`의 거북이를 멈추기 위해 다음 명령을 실행한다.

```
ros2 param set /reg_params move_turtle stop
```

```
ros2 param set /reg_params move_turtle stop
Set parameter successful

```

`turtlesim_node`의 거북이가 멈춘 것을 확인한다.





[튜토리얼 목록](../README.md) 

