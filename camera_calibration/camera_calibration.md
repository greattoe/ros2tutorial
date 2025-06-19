## ar_marker/camera_calibration 



---

**빌드 환경 :**  colcon **/** Ubuntu 20.04 **/** Foxy

---

노트북에 내장된 웹캠으로 AR Marker를 인식시키거나, 영상처리를 통해 거리 등을 측정하려면 이 때 사용되는 카메라의 보정(camera calibration)작업이 필요하다. 카메라의 보정(camera calibration)작업이 완료되면 `~/.ros/camera_info`폴더가 생성되고, 그안에 `camera.yaml`과 같은 카메라설정 파일이 만들어 지는데, 이 설정파일을 생성하는 것이 카메라의 보정(camera calibration)작업의 목적이라고도 할 수 있다. 카메라의 보정(camera calibration)작업을 위해 우선 노트북에 기본으로 내장된 웹캠이나 USB카메라를 입력소스로 이미지토픽을 발행하는 ROS 노드 패키지인 `usb_cam`노드 패키지를 설치한다. (꼭 `usb_cam`패키지가 아니더라도 해당 카메라를 통해 취득한 영상을 이미지 토픽으로 발행할 수 있는 노드 패키지이면 된다.)

`usb_cam` ROS 패키지 설치.

```bash
 sudo apt install ros-foxy-usb-cam
```

이제, 카메라 보정 작업을 위한 `camera calibration` 패키지 설치해야 하는데, 바이너리 설치파일이 제공되지 않으므로 소스코드를 복제하여 빌드해야 한다. 

소스코드를 복제할 워크스페이스의 `src`폴더로 작업경로 변경.

```
cd ~/robot_ws/src
```

소스코드 복제.

```
git clone https://github.com/ros-perception/image_pipeline.git -b foxy
```

빌드를 위한 워크스페이스로 작업경로 변경

```
cd ~/robot_ws
```

빌드(`--packages-select`옵션을 사용하여 `camera_calibration`패키지만 빌드)

```
colcon build --packages-select camera_calibration
```



새로 빌드된 ROS 패키지 정보를 사용자 환경에 반영

```
source ~/robot_ws/install/local_setup.bash
```





[카메라 보정용 패턴 생성 및 다운로드](https://calib.io/pages/camera-calibration-pattern-generator?srsltid=AfmBOoqx8kIfKwjDHXc3WHrMgFvpTKPTOAFZlJFttfvprlgeJQMd-Rra) 

위 사이트에서 **Target Type**은 checker board, 

A4용지 크기가  가로 297(mm) x 세로 210(mm) 인 것과 인쇄 마진을 고려하여 **Board Width**는 277, **Board Height**는 190. **Rows**는 7, **Columns**는 9, **Checker Width**는 25를 입력하고 체커보드 이미지 하단의 [**Save pattern as PDF**]를 클릭하여 생성된 체커보드 PDF파일을 다운로드한다. 

[칼리브레이션 용 8x6 25mm 패턴 다운로드 ](https://drive.google.com/file/d/1V_mACOCYNFdIYNgH_zZnQTWXKQEg4oCj/view?usp=drive_link) 



`usb_cam`노드 실행.

```bash
ros2 run usb_cam usb_cam_node_exe
```

`usb_cam`노드 실행 후, 노드 리스트 확인.


```bash
ros2 node list
/usb_cam
```

`usb_cam`노드 실행 후, 파라메터 리스트 확인.

```bash
ros2 param list 
/usb_cam:
  auto_white_balance
  autoexposure
  autofocus
  brightness
  camera_info_url
  camera_name
  contrast
  exposure
  focus
  frame_id
  framerate
  gain
  image_height
  image_raw.format
  image_raw.jpeg_quality
  image_raw.png_level
  image_width
  io_method
  pixel_format
  saturation
  sharpness
  use_sim_time
  video_device
  white_balance
```

위 `ros2 param list`명령 실행결과로부터 `usb_cam`노드와 관련된24개의 `parameter`목록을 볼 수 있다. 이 들 중 `camera_name`의 값을 다음 명령으로 알아보자.

```bash
gnd0@nt930:~$ ros2 param get /usb_cam camera_name 
String value is: default_cam
```

`camera_name`파라메터 값이 `default_cam`이라는 것을 일단 기억해 둔다.



`camera_calibration` 노드 실행

```
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.025 \
  --camera_name default_cam \
  --ros-args -r image:=/image_raw -r camera_info:=/camera_info

```



| 옵션             | 설명                                                         |
| ---------------- | ------------------------------------------------------------ |
| `--size 8x6`     | 체커보드의 내부 교차점의 개수 (가로 x 세로)                  |
| `--square 0.025` | 사각형 한 칸의 실제 크기 (미터 단위: 2.5cm이면 `0.025`)      |
| `--image_topic`  | 카메라 이미지 토픽 이름 (`/image_raw` 또는 `/camera/image_raw`) |
| `--camera_name`  | 저장될 보정 파일 이름의 prefix (`~/.ros/camera_info/{camera_name}.yaml`) |

카메라 칼리브레이션 노드를 실행한 터미널 창에 다음과 같은 메세지가 출력되고,

```
ros2 run camera_calibration cameracalibrator     --size 8x6     --square 0.025     --camera_name default_cam     --ros-args     -r image:=/image_raw     -r camera_info:=/camera_info
Waiting for service camera/set_camera_info ...
OK
Waiting for service left_camera/set_camera_info ...
OK
Waiting for service right_camera/set_camera_info ...
OK
```



다음과 같은 카메라 칼리브레이션 화면이 나타나면 준비한 체커보드를 카메라에 비쳐준다.

<img src="/home/gnd0/ros2tutorial/camera_calibration/img/monocular_camera_calibrate_1.png" width="100%">

아래 메세지와 함께 첫 번째 `sample` 추가된 후, 

```bash
Waiting for service //set_camera_info ...
OK
*** Added sample 1, p_x = 0.534, p_y = 0.482, p_size = 0.398, skew = 0.036
```

`checkerboard` 를 이리 저리 움직임에 따라 

<img src="/home/gnd0/ros2tutorial/camera_calibration/img/monocular_camera_calibrate_2.png" width="49.5%"> <img src="/home/gnd0/ros2tutorial/camera_calibration/img/monocular_camera_calibrate_3.png" width="49.5%"> 

<img src="/home/gnd0/ros2tutorial/camera_calibration/img/monocular_camera_calibrate_4.png" width="49.5%"> <img src="/home/gnd0/ros2tutorial/camera_calibration/img/monocular_camera_calibrate_5.png" width="49.5%"> 

`sample` 이 계속 추가된다. 

```bash
*** Added sample 2, p_x = 0.675, p_y = 0.434, p_size = 0.404, skew = 0.048
*** Added sample 3, p_x = 0.383, p_y = 0.481, p_size = 0.346, skew = 0.026
*** Added sample 4, p_x = 0.433, p_y = 0.333, p_size = 0.348, skew = 0.044
*** Added sample 5, p_x = 0.604, p_y = 0.324, p_size = 0.352, skew = 0.072
*** Added sample 6, p_x = 0.631, p_y = 0.513, p_size = 0.340, skew = 0.062
*** Added sample 7, p_x = 0.510, p_y = 0.593, p_size = 0.336, skew = 0.029
*** Added sample 8, p_x = 0.413, p_y = 0.684, p_size = 0.337, skew = 0.016
*** Added sample 9, p_x = 0.246, p_y = 0.640, p_size = 0.342, skew = 0.019
*** Added sample 10, p_x = 0.126, p_y = 0.726, p_size = 0.362, skew = 0.013
*** Added sample 11, p_x = 0.236, p_y = 0.779, p_size = 0.366, skew = 0.079
*** Added sample 12, p_x = 0.559, p_y = 0.051, p_size = 0.377, skew = 0.056
*** Added sample 13, p_x = 0.449, p_y = 0.120, p_size = 0.372, skew = 0.028
*** Added sample 14, p_x = 0.381, p_y = 0.208, p_size = 0.365, skew = 0.076
*** Added sample 15, p_x = 0.306, p_y = 0.152, p_size = 0.365, skew = 0.005
*** Added sample 16, p_x = 0.550, p_y = 0.228, p_size = 0.412, skew = 0.066
*** Added sample 17, p_x = 0.412, p_y = 0.863, p_size = 0.396, skew = 0.015
*** Added sample 18, p_x = 0.359, p_y = 0.380, p_size = 0.397, skew = 0.001
*** Added sample 19, p_x = 0.516, p_y = 0.714, p_size = 0.391, skew = 0.001
*** Added sample 20, p_x = 0.264, p_y = 0.288, p_size = 0.422, skew = 0.013
*** Added sample 21, p_x = 0.464, p_y = 0.278, p_size = 0.433, skew = 0.002
*** Added sample 22, p_x = 0.580, p_y = 0.625, p_size = 0.431, skew = 0.015
*** Added sample 23, p_x = 0.575, p_y = 0.836, p_size = 0.427, skew = 0.008
*** Added sample 24, p_x = 0.436, p_y = 0.566, p_size = 0.413, skew = 0.005
*** Added sample 25, p_x = 0.582, p_y = 0.411, p_size = 0.425, skew = 0.114
*** Added sample 26, p_x = 0.549, p_y = 0.861, p_size = 0.305, skew = 0.047
*** Added sample 27, p_x = 0.574, p_y = 0.441, p_size = 0.290, skew = 0.004
*** Added sample 28, p_x = 0.571, p_y = 0.689, p_size = 0.284, skew = 0.027
*** Added sample 29, p_x = 0.667, p_y = 0.286, p_size = 0.456, skew = 0.066
*** Added sample 30, p_x = 0.471, p_y = 0.960, p_size = 0.435, skew = 0.008
*** Added sample 31, p_x = 0.501, p_y = 0.390, p_size = 0.470, skew = 0.026
*** Added sample 32, p_x = 0.502, p_y = 0.140, p_size = 0.485, skew = 0.047
*** Added sample 33, p_x = 0.568, p_y = 0.417, p_size = 0.370, skew = 0.245
*** Added sample 34, p_x = 0.515, p_y = 0.375, p_size = 0.344, skew = 0.132
*** Added sample 35, p_x = 0.517, p_y = 0.284, p_size = 0.445, skew = 0.158
*** Added sample 36, p_x = 0.576, p_y = 0.168, p_size = 0.396, skew = 0.182
*** Added sample 37, p_x = 0.603, p_y = 0.031, p_size = 0.425, skew = 0.201
*** Added sample 38, p_x = 0.231, p_y = 0.452, p_size = 0.394, skew = 0.004
*** Added sample 39, p_x = 0.800, p_y = 0.096, p_size = 0.399, skew = 0.126
*** Added sample 40, p_x = 0.775, p_y = 0.358, p_size = 0.391, skew = 0.032
*** Added sample 41, p_x = 0.857, p_y = 0.467, p_size = 0.423, skew = 0.000
```

충분한 `sample` 이 추가되고 나면 카메라 칼리브레이션 화면이 멈추고 `CALIBRATE` 버튼이 활성화된다. 

<img src="/home/gnd0/ros2tutorial/camera_calibration/img/monocular_camera_calibrate_6.png" width="100%">

활성화된 `CALIBRATE` 버튼을 클릭하면 아래 메세지와 함께 카메라 칼리브레이션 화면이 검게 변하기를 반복한다.

```bash
**** Calibrating ****
```

카메라 칼리브레이션이 완료되면 그 결과가 터미널 창에 출력되고,

```bash
mono pinhole calibration...
D =  [0.10264789613758656, -0.022657587841963832, -0.055046049219766455, 0.00024937145988498943, 0.0]
K =  [521.5551377985744, 0.0, 305.83346734373504, 0.0, 533.3983687542734, 173.73902518367848, 0.0, 0.0, 1.0]
R =  [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P =  [556.9845581054688, 0.0, 306.0941488417666, 0.0, 0.0, 534.4867553710938, 150.4472439842648, 0.0, 0.0, 0.0, 1.0, 0.0]
None
# oST version 5.0 parameters


[image]

width
640

height
480

[default_cam]

camera matrix
521.555138 0.000000 305.833467
0.000000 533.398369 173.739025
0.000000 0.000000 1.000000

distortion
0.102648 -0.022658 -0.055046 0.000249 0.000000

rectification
1.000000 0.000000 0.000000
0.000000 1.000000 0.000000
0.000000 0.000000 1.000000

projection
556.984558 0.000000 306.094149 0.000000
0.000000 534.486755 150.447244 0.000000
0.000000 0.000000 1.000000 0.000000

```

다음 그림과 같이 카메라 칼리브레이션 화면에 `SAVE` ,  `COMMIT` 버튼까지 활성화 된 것을 볼 수 있다.

<img src="/home/gnd0/ros2tutorial/camera_calibration/img/monocular_camera_calibrate_7.png" width="100%">

이 때 `SAVE` 버튼을 클릭하면 다음 메세지가 출력되면서 카메라 칼리브레이션 결과가 `/tmp/calibrationdata.tar.gz` 파일로 저장된다. 

```
('Wrote calibration data to', '/tmp/calibrationdata.tar.gz')

```



`COMMIT` 버튼을 클릭하면 최종결과의 화면 출력과 동시에 그 결과를 `~/.ros/camera_info/camera.yaml` 에 기록 되어야하지만 `cameracalibrator`노드의 버그 때문인지 다음과 같은 에러메세지만 출력된다.

```bash
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

[ERROR]: Not available

```



하지만 `SAVE`버튼을 클릭했을 때 저장된 `/tmp/calibrationdata.tar.gz`파일로부터 이요하여 카메라 보정 파일을 추출해낼 수 있다.

`/mp`폴더로 경로 변경

```
cd /tmp
```



`tmp/calibrationdata.tar.gz`파일 압축해제

```
tar -xvzf ./calibrationdata.tar.gz
```

`tmp/calibrationdata.tar.gz`파일의 압축을 해제결과 중,  칼리브레이션 작업 중 추가된 샘플 이미지들과 함께`ost.txt`, `ost.yaml`파일을 확인할 수 있다. 이들 중 `ost.yaml`파일을 `~/.ros/camera_info/default_cam.yaml`로 복사하기 위해 `~/.ros/camera_info`폴더를 생성한다.

```bash
mkdir ~/.ros/camera_info
```



`ost.yaml`을 `~/.ros/camera_info/default_cam.yaml`로 복사 복사되는 파일명 `default_cam`은 앞서 `usb_cam`노드의 `camera_name`파라메터 값을 참조한 것이다.

```bash
 cp ost.yaml ~/.ros/camera_info/default_cam.yaml
```



`usb_cam`노드를 종료 후, 다시실행 하면 아래와 같이 구동되는 것을 볼 수 있다.

```
ros2 run usb_cam usb_cam_node_exe 
[INFO]: camera_name value: default_cam
[WARN]: framerate: 30.000000
[INFO]: using default calibration URL
[INFO]: camera calibration URL: file:///home/gnd0/.ros/camera_info/default_cam.yaml
[INFO]: Starting 'default_cam' (/dev/video0) at 640x480 via mmap (yuyv) at 30 FPS
[INFO]: This devices supproted formats:
[INFO]: 	Motion-JPEG: 1920 x 1080 (30 Hz)
[INFO]: 	Motion-JPEG: 1280 x 720 (30 Hz)
[INFO]: 	Motion-JPEG: 960 x 540 (30 Hz)
[INFO]: 	Motion-JPEG: 848 x 480 (30 Hz)
[INFO]: 	Motion-JPEG: 640 x 480 (30 Hz)
[INFO]: 	Motion-JPEG: 640 x 360 (30 Hz)
[INFO]: 	Motion-JPEG: 424 x 240 (30 Hz)
[INFO]: 	Motion-JPEG: 352 x 288 (30 Hz)
[INFO]: 	Motion-JPEG: 320 x 240 (30 Hz)
[INFO]: 	Motion-JPEG: 320 x 180 (30 Hz)
[INFO]: 	YUYV 4:2:2: 1920 x 1080 (5 Hz)
[INFO]: 	YUYV 4:2:2: 1280 x 720 (10 Hz)
[INFO]: 	YUYV 4:2:2: 960 x 540 (15 Hz)
[INFO]: 	YUYV 4:2:2: 848 x 480 (20 Hz)
[INFO]: 	YUYV 4:2:2: 640 x 480 (30 Hz)
[INFO]: 	YUYV 4:2:2: 640 x 360 (30 Hz)
[INFO]: 	YUYV 4:2:2: 424 x 240 (30 Hz)
[INFO]: 	YUYV 4:2:2: 352 x 288 (30 Hz)
[INFO]: 	YUYV 4:2:2: 320 x 240 (30 Hz)
[INFO]: 	YUYV 4:2:2: 320 x 180 (30 Hz)
[INFO]: Setting 'brightness' to 50
unknown control 'white_balance_temperature_auto'

[INFO]: Setting 'white_balance_temperature_auto' to 1
[INFO]: Setting 'exposure_auto' to 3
unknown control 'exposure_auto'

[INFO]: Setting 'focus_auto' to 0
unknown control 'focus_auto'

[INFO]: Timer triggering every 33 ms
```



이 번에는 `~/.ros/camera_info/default_cam.yaml`파일을 삭제하거나 파일명 변경 하거나서 `usb_cam`노드를 종료 후, 다시실행 하면 아래와 같이 구동되는 것을 볼 수 있다.



```
ros2 run usb_cam usb_cam_node_exe 
[INFO]: camera_name value: default_cam
[WARN]: framerate: 30.000000
[INFO]: using default calibration URL
[INFO]: camera calibration URL: file:///home/gnd0/.ros/camera_info/default_cam.yaml
[ERROR]: Unable to open camera calibration file [/home/gnd0/.ros/camera_info/default_cam.yaml]
[WARN]: Camera calibration file /home/gnd0/.ros/camera_info/default_cam.yaml not found
[INFO]: Starting 'default_cam' (/dev/video0) at 640x480 via mmap (yuyv) at 30 FPS
[INFO]: This devices supproted formats:
[INFO]: 	Motion-JPEG: 1920 x 1080 (30 Hz)
[INFO]: 	Motion-JPEG: 1280 x 720 (30 Hz)
[INFO]: 	Motion-JPEG: 960 x 540 (30 Hz)
[INFO]: 	Motion-JPEG: 848 x 480 (30 Hz)
[INFO]: 	Motion-JPEG: 640 x 480 (30 Hz)
[INFO]: 	Motion-JPEG: 640 x 360 (30 Hz)
[INFO]: 	Motion-JPEG: 424 x 240 (30 Hz)
[INFO]: 	Motion-JPEG: 352 x 288 (30 Hz)
[INFO]: 	Motion-JPEG: 320 x 240 (30 Hz)
[INFO]: 	Motion-JPEG: 320 x 180 (30 Hz)
[INFO]: 	YUYV 4:2:2: 1920 x 1080 (5 Hz)
[INFO]: 	YUYV 4:2:2: 1280 x 720 (10 Hz)
[INFO]: 	YUYV 4:2:2: 960 x 540 (15 Hz)
[INFO]: 	YUYV 4:2:2: 848 x 480 (20 Hz)
[INFO]: 	YUYV 4:2:2: 640 x 480 (30 Hz)
[INFO]: 	YUYV 4:2:2: 640 x 360 (30 Hz)
[INFO]: 	YUYV 4:2:2: 424 x 240 (30 Hz)
[INFO]: 	YUYV 4:2:2: 352 x 288 (30 Hz)
[INFO]: 	YUYV 4:2:2: 320 x 240 (30 Hz)
[INFO]: 	YUYV 4:2:2: 320 x 180 (30 Hz)
[INFO]: Setting 'brightness' to 50
unknown control 'white_balance_temperature_auto'

[INFO]: Setting 'white_balance_temperature_auto' to 1
[INFO]: Setting 'exposure_auto' to 3
unknown control 'exposure_auto'

[INFO]: Setting 'focus_auto' to 0
unknown control 'focus_auto'

[INFO]: Timer triggering every 33 ms
```



`ros2_aruco_node`를 구동해봐도제대로 된 `~/.ros/camera_info/default_cam.yaml`파일의 존재 유무에 따라 `/aruco_makers`토픽 내용이 차이가 나는 것을 확인할 수 있다. 



---



[튜토리얼 목록](../README.md) 







