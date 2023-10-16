## rospy_tutorial/ Tutorials/ WritingService & Client



---

## 서비스 서버와 클라이언트 작성 

**출처 :**  <https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html>

**튜토리얼 레벨 :**  초급

**빌드 환경 :**  colcon **/** Ubuntu 20.04 **/** Foxy

---

서비스를 이용한 노드 사이의 통신에서 서비스를 요청하는 노드를 클라이언트 노드, 클라이언트의 요청에 응답하는 노드를 서비스 노드라 한다. 이 때 서비스 요청 응답에 대한 데이터 구조는 `.srv` 파일(서비스 파일)에 의해 정의된다. 작성할 예제는 클라이언트 노드가 두 정수의 합을 요청하면, 서비스 노드가 그 합을 구해 응답하는 예제이다.

패키지 생성을 위해 작업 경로를 워크스페이스`~/robot_ws` 의 `src` 폴더로 변경한다. 

```
cd ~/robot_ws/src
```

`rclpy` 와 `std_msgs` 에 의존성을 갖는 패키지 `py_srvcli` 생성

```
ros2 pkg create py_srvcli --build-type ament_python --dependencies rclpy example_interfaces
```

`~/robot_ws/src/ex_rclpy/py_srvcli`폴더로 작업 경로 변경

```
cd ~/robot_ws/src/py_srvcli/py_srvcli
```

`ls` 명령으로 작업경로에 `__init__.py`파일의 존재를 확인한다.

```
ls__init__.py
```

`__init__.py`파일과 같은 경로에 서비스 노드  `service_member_function.py`를 다음과 같이 작성한다. 

```
gedit service_member_function.py &
```

```python
from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
```



##### `setup.py` 에 작성한 서비스 노드의`entry_point` 추가

작성한 코드를 저장 후, 한 단계 상위 디렉토리로 경로를 변경 후, `setup.py`파일을 편집한다. 

```
cd ..
```

```
gedit setup.py &
```

```python
from setuptools import setup

package_name = 'py_srvcli'

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
        'service = py_srvcli.service_member_function:main',
        ],
    },
)
```

위 내용 아래 `'console_scripts'` 필드의 대괄호 안에 다음과 같이 한 줄을 추가한다. 

```python
entry_points={
        'console_scripts': [
        'service = py_srvcli.service_member_function:main',
        ],
},
```

##### 서비스클라이언트 작성

두 정수의 합을 구해주는 서비스를 제공하는 `service_member_function.py`노드에게 서비스를 요청하는 `client_member_function.py`노드 작성을 위해 `service_member_function.py`노드를 작성했던 폴더로 경로를 변경한다. 

```
cd py_srvcli
```

같은 경로에 `__init__.py`와 `service_member_function.py`가 있는지 확인한 후 `client_member_function.py`노드를 작성한다. 

```
ls
__init__.py  service_member_function.py
```

```
gedit client_member_function.py
```

```python
import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

```
<description>Examples of minimal publisher/subscriber using rclpy</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```



##### 서비스 노드의 `entry_point` 추가

앞서 작성한 `service_member_function.py`노드를 `setup.py`파일의 `'console_scripts'` 필드에 추가한 것과 같이 `client_member_function.py` 노드도 추가한다. 이를 위해 작업 경로를 한단계 상위 폴더로 변경한다. 

```
cd ..
```

`setup.py`파일의 `entry_points` 필드를 다음과 같이 편집 후, 저장한다.

```
gedit setup.py &
```

```python
from setuptools import setup

package_name = 'py_srvcli'

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
        'service = py_srvcli.service_member_function:main',
        'client = py_srvcli.client_member_function:main',
        ],
    },
)
```



`package.xml` 파일의 위 `<license>` 태그 뒤에 아래의 `<exec_depend>` 추가 

```
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```



##### 빌드 및 실행

빌드를 위해 워크스페이스로 경로를 변경한다.

```
cd ~/robot_ws
```

워크 스페이스의 모든 패키지 빌드

```
colcon build --symlink-install
```

특정 패키지만 선택하여 빌드

```
colcon build --symlink-install --packages-select py_srvcli
```

지금 빌드된 패키지에 대한 정보 반영을 위해 아래 명령을 실행한다.

```
. ~/robot_ws/install/local_setup.bash
```

서비스노드 `service` 실행

```
ros2 run py_srvcli service
```

서비스 노드가 실행되었지만 서비스를 요청하는 클라이언트가 없어 아무 변화가 없다. 

클라이언트 노드 `client` 실행

```
os2 run py_srvcli client 2 3
[INFO] [1697413310.060098558] [minimal_client_async]: Result of add_two_ints: for 2 + 3 = 5
```



이 때 서비스 노드가 실행된 창을 확인해 보면 다음과 같이 표시되는 것을 확인할 수 있다. 

```
ros2 run py_srvcli service 
[INFO] [1697413310.052516210] [minimal_service]: Incoming request
a: 2 b: 3
```













[튜토리얼 목록](./README.md) 







