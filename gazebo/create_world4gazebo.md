## Gazebo 시뮬레이터를 위한 커스텀 world 만들기

**빌드 환경 :**  colcon **/** Ubuntu 20.04 **/** Foxy

가제보 구동

```bash
ros2 launch gazebo_ros gazebo.launch.py
```





## 전체 작업 흐름

##### 1. 패키지 이름: `my_turtlebot3_world`

##### 2. 월드 파일 이름: `my_turtlebot3_world.world`

##### 3. 런치 파일 이름: `my_turtlebot3_world.launch.py`



---



1. ✅ 1. 패키지 생성

```bash
cd ~/robot_ws/src
ros2 pkg create my_turtlebot3_world --build-type ament_python --dependencies gazebo_ros turtlebot3_description turtlebot3_gazebo

```



✅ 2. 디렉토리/파일 구성

```bash
cd my_turtlebot3_world
mkdir launch worlds resource my_turtlebot3_world
touch my_turtlebot3_world/__init__.py
touch resource/my_turtlebot3_world

```



✅ 3. world 파일 작성

```bash
gedit worlds/my_turtlebot3_world.world
```



```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="my_turtlebot3_world">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://box</uri>
      <pose>1 1 0.5 0 0 0</pose>
    </include>
  </world>
</sdf>

```





✅ 4. 런치 파일 작성

```bash
gedit launch/my_turtlebot3_world.launch.py
```




```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    # ✅ 내 custom world 경로
    world = os.path.join(
        get_package_share_directory('my_turtlebot3_world'),
        'worlds',
        'my_turtlebot3_world.world'
    )

    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            )
        ),

        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
    ])

```



✅ 5. `setup.py`

```python
from setuptools import setup

package_name = 'my_turtlebot3_world'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name + '/launch', ['launch/my_turtlebot3_world.launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/my_turtlebot3_world.world']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Launch TurtleBot3 in my custom Gazebo world',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)

```



✅ 6. `setup.cfg`

```bash
[develop]
script_dir=$base/lib/my_turtlebot3_world
[install]
install_scripts=$base/lib/my_turtlebot3_world

```



✅ 7. `package.xml`

```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_turtlebot3_world</name>
  <version>0.0.1</version>
  <description>Launch TurtleBot3 in a custom Gazebo world</description>

  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>gazebo_ros</exec_depend>
  <exec_depend>turtlebot3_description</exec_depend>
  <exec_depend>turtlebot3_gazebo</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>

```



✅ 8. 빌드

```bash
cd ~/robot_ws
colcon build --packages-select my_turtlebot3_world
source install/setup.bash

```



✅ 9. 실행

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch my_turtlebot3_world my_turtlebot3_world.launch.py

```



빌드

```bash
cd ~/robot_ws
colcon build --packages-select turtlebot3_myworld
source install/setup.bash

```



실행

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_myworld turtlebot3_world.launch.py

```















```bash
touch launch/bringup_world.launch.py
```



저장한 world를 생성된 패키지 폴더의 `world`폴더에 복사 

```bash
cp ~/*.world ~/robot_ws/src/my_gazebo_world/worlds/
```



성된 패키지 폴더의 `launch`폴더의 `launch/bringup_world.launch.py`파일 편집

```bash
gedit  ~/robot_ws/src/my_gazebo_world/launch/bringup_world.launch.py &
```



```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    world_path = PathJoinSubstitution([
        FindPackageShare('my_gazebo_world'),
        'worlds',
        'test0620.world'
    ])

    return LaunchDescription([
        # Gazebo 실행
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # TurtleBot3 모델 스폰
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'turtlebot3',
                '-file', '/usr/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf',
                '-x', '0', '-y', '0', '-z', '0.1'
            ],
            output='screen'
        )
    ])

```



`setup.py` 수정

```python
from setuptools import setup
import os
from glob import glob

package_name = 'my_gazebo_world'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='Launch TurtleBot3 in custom Gazebo world',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)

```











[튜토리얼 목록](../README.md) 







