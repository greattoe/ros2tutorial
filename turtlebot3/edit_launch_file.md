## Turtlebot3/ Tutorials/edit_launch_file



---

## `turtlebot3_bringup`패키지의 `launch`파일 수정

**튜토리얼 레벨 :**  Intermediate(중급)

**이 튜토리얼 작성 환경 :**  colcon **/** Ubuntu 20.04 **/** Foxy

**튜토리얼 목록 :** [README.md](../../README.md)

------

터틀봇3 구동 `launch`파일은 라즈베리파이의 `~/turtlebot3_ws/src/turtlebot3/turtlebot3_bringup/launch/robot.launch.py`파일이며 그 내용은 다음과 같다.

``` python
#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')

    tb3_param_dir = LaunchConfiguration(
        'tb3_param_dir',
        default=os.path.join(
            get_package_share_directory('turtlebot3_bringup'),
            'param',
            TURTLEBOT3_MODEL + '.yaml'))

    lidar_pkg_dir = LaunchConfiguration(
        'lidar_pkg_dir',
        default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'usb_port',
            default_value=usb_port,
            description='Connected USB port with OpenCR'),

        DeclareLaunchArgument(
            'tb3_param_dir',
            default_value=tb3_param_dir,
            description='Full path to turtlebot3 parameter file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/turtlebot3_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, '/hlds_laser.launch.py']),
            launch_arguments={'port': '/dev/ttyUSB0', 'frame_id': 'base_scan'}.items(),
        ),

        Node(
            package='turtlebot3_node',
            executable='turtlebot3_ros',
            parameters=[tb3_param_dir],
            arguments=['-i', usb_port],
            output='screen'),
    ])
```



미션 수행 시 AR-Marker를 활용하거나 영상처리 작업이 필요할 경우 위 `turtlebot3_bringup` 패키지의 `robot.launch.py`를 실행 후, 추가로 카메라 구동 노드를 실행해야만 한다. 이를 위해선 별도의 터미널 창에서 터틀봇3에 `ssh`원격 접속을 하여 다음 명령을 실행하여 `raspicam2_node`를 구동해야 한다.

```bash
ros2 run raspicam2 raspicam2_node --ros-args --params-file `ros2 pkg prefix raspicam2`/share/raspicam2/cfg/params.yaml
```



여기서  `turtlebot3_bringup` 패키지의 `robot.launch.py`파일을 `bringup` 작업 후에 카메라도 구동하도록 수정 편집해보자.

기존 `robot.launch.py`파일 하단의 아래 내용 뒤에

```python
Node(
            package='turtlebot3_node',
            executable='turtlebot3_ros',
            parameters=[tb3_param_dir],
            arguments=['-i', usb_port],
            output='screen'),
```



다음 내용을 추가하여 `camera.launch.py`로 저장한다.

```python
 Node(
            package='turtlebot3_node',
            executable='turtlebot3_ros',
            parameters=[tb3_param_dir],
            arguments=['-i', usb_port],
            output='screen'),
```



다음은 편집이 완료된 `camera.launch.py`파일의 내용이다.

```python
       #!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')

    tb3_param_dir = LaunchConfiguration(
        'tb3_param_dir',
        default=os.path.join(
            get_package_share_directory('turtlebot3_bringup'),
            'param',
            TURTLEBOT3_MODEL + '.yaml'))

    lidar_pkg_dir = LaunchConfiguration(
        'lidar_pkg_dir',
        default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'usb_port',
            default_value=usb_port,
            description='Connected USB port with OpenCR'),

        DeclareLaunchArgument(
            'tb3_param_dir',
            default_value=tb3_param_dir,
            description='Full path to turtlebot3 parameter file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/turtlebot3_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, '/hlds_laser.launch.py']),
            launch_arguments={'port': '/dev/ttyUSB0', 'frame_id': 'base_scan'}.items(),
        ),

        Node(
            package='turtlebot3_node',
            executable='turtlebot3_ros',
            parameters=[tb3_param_dir],
            arguments=['-i', usb_port],
            output='screen'
        ),

        Node(
            package='raspicam2',
            executable='raspicam2_node',
            parameters=[os.path.join(
                get_package_share_directory('raspicam2'),
                'cfg',
                'params.yaml')],
            output='screen'
        ),
    ])


```



`turtlebot3_bringup`패키지에 추가된 `camera.launch.py`파일을 반영하기 위해선 `turtlebot3_bringup`패키지를 빌드해야 한다.

`turtlebot3_bringup`패키지 빌드를 위해 터틀봇3의 라즈베리파이에 `ssh` 원격 접속 후 작업경로를 `~/turtlebot3_ws`로 변경한다. 

```bash
cd ~/turtlebot3_ws
```



`turtlebot3_bringup`패키지 빌드

```bash
colcon build --packages-select turtlebot3_bringup
```



빌드 결과물의 실행환경 설정 스크립트 소싱

```bash
source ./install/local_setup.bash
```



수정된 `robot.launch.py`구동후, 원격 컴퓨터(remote PC:노트북)에서 `ros2 topic list`를 실행하여 `turtlebot3_bringup`구동 시 나타나는 토픽들과, `raspicam2_node`구동 시 나타나는 토픽이 모두 출력되는 지 확인한다.

```bash
$ ros2 topic list 
/battery_state #------------ turtlebot3_bringup
/camera/image/camera_info #- raspicam2_node
/camera/image/compressed # - raspicam2_node
/cmd_vel# ------------------ turtlebot3_bringup
/imu# ---------------------- turtlebot3_bringup
/joint_states# ------------- turtlebot3_bringup
/magnetic_field# ----------- turtlebot3_bringup
/odom# --------------------- turtlebot3_bringup
/parameter_events# --------- turtlebot3_bringup
/robot_description# -------- turtlebot3_bringup
/rosout
/scan
/sensor_state
/tf
/tf_static
```

임무장비로 설치한 리프트 제어노드를 실행 하려면 다음 명령을 실행해야 한다.

``` bash
ros2 run lift_ctrl lift_ctrl
```

 `camera.launch.py`파일에 다음 내용을 추가하여 `~/turtlebot3_ws/src/turtlebot3/turtlebot3_bringup/launch/lift.launch.py`로 저장한다.

```python


        Node(
            package='lift_ctrl',
            executable='lift_ctrl',
            output='screen'
        ),
    ])
```



편집이 완료된  `camera.launch.py`는 다음과 같다.

```python
#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0 (the "License");
# ...

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')

    tb3_param_dir = LaunchConfiguration(
        'tb3_param_dir',
        default=os.path.join(
            get_package_share_directory('turtlebot3_bringup'),
            'param',
            TURTLEBOT3_MODEL + '.yaml'))

    lidar_pkg_dir = LaunchConfiguration(
        'lidar_pkg_dir',
        default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'usb_port',
            default_value=usb_port,
            description='Connected USB port with OpenCR'),

        DeclareLaunchArgument(
            'tb3_param_dir',
            default_value=tb3_param_dir,
            description='Full path to turtlebot3 parameter file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/turtlebot3_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, '/hlds_laser.launch.py']),
            launch_arguments={'port': '/dev/ttyUSB0', 'frame_id': 'base_scan'}.items(),
        ),

        Node(
            package='turtlebot3_node',
            executable='turtlebot3_ros',
            parameters=[tb3_param_dir],
            arguments=['-i', usb_port],
            output='screen'
        ),

        Node(
            package='raspicam2',
            executable='raspicam2_node',
            parameters=[os.path.join(
                get_package_share_directory('raspicam2'),
                'cfg',
                'params.yaml')],
            output='screen'
        ),

        Node(
            package='lift_ctrl',
            executable='lift_ctrl',
            output='screen'
        ),
    ])

```



빌드를 위해 `~/turtlebot3_ws`로 작업 경로 변경.

```
cd ~/turtlebot3_ws
```

`turtlebot3_bringup`패키지만 빌드

```
colcon build --symlink-install --packages-select turtlebot3_bringup
```

빌드결과 반영

```
source ./install/local_setup.bash
```

`lift.launch.py` 실행

```
ros2 launch turtlebot3_bringup lift.launch.py
```

Remote PC(노트북)에서 `topic list`확인

```bash
 ros2 topic list 
/battery_state#<<--------------turtlebor3_bringup
/camera/image/camera_info# <<--raspicam2_node
/camera/image/compressed# <<---raspicam2_node
/cmd_vel#<<--------------------turtlebor3_bringup
/imu# <<-----------------------turtlebor3_bringup
/joint_states#<<---------------turtlebor3_bringup
/lift_msg# <<------------------lift_ctrl
/magnetic_field#<<-------------turtlebor3_bringup
/odom#<<-----------------------turtlebor3_bringup
/parameter_events#<<-----------ros2 topic for managing system and status monitoring 
/robot_description#<<----------turtlebor3_bringup
/rosout#<<---------------------ros2 topic for managing system and status monitoring 
/scan#<<-----------------------turtlebor3_bringup
/sensor_state#<<---------------turtlebor3_bringup
/tf#<<-------------------------turtlebor3_bringup
/tf_static#<<------------------turtlebor3_bringup
```

`turtlebot3_bringup`관련토픽, `raspicam2_node`관련 토픽, `lift_ctrl`노드 관련 토픽들을 모두 확인할 수 있다.





[튜토리얼 목록 열기](../README.md)

