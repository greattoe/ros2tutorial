## rclpy Tutorials Writing simple Publisher & Subscriber



---

## 퍼블리셔와 서브스크라이버 작성 

**출처 :**  <https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html>

**튜토리얼 레벨 :**  초급

**빌드 환경 :**  colcon **/** Ubuntu 22.04 **/** Humble

---

#### 퍼블리셔 작성

작업 경로를 워크스페이스`~/robot_ws` 의 `src` 폴더로 변경한다. 

```
cd ~/robot_ws/src
```

`rclpy` 와 `std_msgs` 에 의존성을 갖는 패키지 `py_pubsub` 생성

```
ros2 pkg create py_pubsub --build-type ament_python
```

`~/robot_ws/src/pypubsub/py_pubsub`폴더로 작업 경로 변경

```
cd ~/robot_ws/src/py_pubsub/py_pubsub
```

`ls` 명령으로 작업경로에 `__init__.py`파일의 존재를 확인한다.

```
ls__init__.py
```

`__init__.py`파일과 같은 경로에 토픽 퍼블리셔 노드  `example_pub.py`를 작성한다. 

```
gedit example_pub.py &
```

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class ExamplePub(Node):

    def __init__(self):
        super().__init__('example_pub')
        self.publisher_ = self.create_publisher(String, 'hello', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i = self.i + 1


def main(args=None):
    rclpy.init(args=args)

    node = MinimalPublisher()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```



##### 작성한 퍼블리셔 노드의`entry_point` 추가

작성한 코드를 저장 후, 한 단계 상위 디렉토리로 경로를 변경 후, `setup.py`파일을 편집한다. 

```
cd ..
```

```
gedit setup.py &
```

```python
setuptools import find_packages
from setuptools import setup

package_name = 'py_pusub'

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
```

위 내용 아래 `'console_scripts'` 필드의 대괄호 안에 다음과 같이 한 줄을 추가한다. 

```python
entry_points={
        'console_scripts': [
                'example_pub = py_pubsub.example_pub:main',
        ],
},
```





#### 서브스크라이버 작성

`minimal_pub.py`노드가 발행하는 토픽을 구독하는 구독자 노드 `example_sub.py`노드 작성을 위해 `example_pub.py`노드를 작성했던 폴더로 경로를 변경한다. 

```bash
cd pypubsub
```

같은 경로에 `__init__.py`와 `minimal_pub.py`가 있는지 확인한 후 `minimal_sub.py`노드를 작성한다. 

```bash
ls
__init__.py  minimal_pub.py
```

```bash
gedit minimal_sub.py &
```

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.create_subscription( String, 'hello', 10)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    node = MinimalSubscriber()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```



##### `setup.py`파일에 서브스크라이버 노드의 `entry_point` 추가

앞서 작성한 `example_pub.py`노드를 `setup.py`파일의 `'console_scripts'` 필드에 추가한 것과 같이 `example_sub.py` 노드도 추가한다. 이를 위 작업 경로를 한 단계 상위 폴더( `~/robot_ws/src/py_pubsub` )로 변경한다. 

```
cd ..
```

`setup.py`파일의 `entry_points` 필드를 다음과 같이 편집 후, 저장한다.

```
gedit setup.py &
```

```python
setuptools import find_packages
from setuptools import setup

package_name = 'pypusub'

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
                'example_pub   = py_pubsub.example_pub:main',
                'example_sub   = py_pubsub.example_sub:main',
        ],
},
```



##### 빌드 및 실행

빌드를 위해 워크스페이스로 경로를 변경한다.

```
cd ~/robot_ws
```

워크 스페이스의 노드 패키지 빌드

```
colcon build --symlink-install
```

특정 패키지만 선택하여 빌드

```
colcon build --symlink-install --packages-select py_pubsub
```

지금 빌드된 패키지에 대한 정보 반영을 위해 아래 명령을 실행한다.

```
. ~/robot_ws/install/local_setup.bash
```

퍼블리셔노드 `example_pub` 실행

```
ros2 run py_pubsub example_pub
```

```
ros2 run py_pubsub example_pub
[INFO]: Publishing: "Hello World: 0"
[INFO]: Publishing: "Hello World: 1"
[INFO]: Publishing: "Hello World: 2"
[INFO]: Publishing: "Hello World: 3"
[INFO]: Publishing: "Hello World: 4"
[INFO]: Publishing: "Hello World: 5"
[INFO]: Publishing: "Hello World: 6"
[INFO]: Publishing: "Hello World: 7"
```



구독자 노드 `example_sub` 실행

```
ros2 run py_pubsub example_sub
```

```
ros2 run py_pubsub example_sub 
[INFO]: I heard: "Hello World: 0"
[INFO]: I heard: "Hello World: 1"
[INFO]: I heard: "Hello World: 2"
[INFO]: I heard: "Hello World: 3"
[INFO]: I heard: "Hello World: 4"
[INFO]: I heard: "Hello World: 5"
[INFO]: I heard: "Hello World: 6"
[INFO]: I heard: "Hello World: 7"
```





[튜토리얼 목록](../README.md) 







