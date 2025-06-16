## ar_marker/camera_calibration 



---

## Camera 간의 상호 변환  



**빌드 환경 :**  colcon **/** Ubuntu 20.04 **/** Foxy

---

카메라를 이용해 AR Marker를 인식하고, 인식된 마커와 카메라 사이의 거리등의 정확한 위치관계 정보를 취득하려면 우선 마커 인식에 사용되는 카메라 보정(camera calibration)작업이 필요하다.

`usb_cam` ROS 패키지 설치.

```bash
 sudo apt install ros-foxy-usb-cam
```

camera calibration 패키지 설치.

```bash
sudo apt install ros-foxy-camera-calibration
```



`usb_cam`노드 실행.

```bash
ros2 launch usb_cam demo_launch.py
```

`usb_cam`노드 실행 후, 토픽 리스트 확인.


```bash
ros2 topic list 
/camera_info
/image_raw # <--------------
/image_raw/compressed
/image_raw/compressedDepth
/image_raw/theora
/parameter_events
/rosout
```



camera_calibration 실행

```
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 --camera_name default_cam --image_topic /image_raw

```



| 옵션             | 설명                                                         |
| ---------------- | ------------------------------------------------------------ |
| `--size 8x6`     | 체커보드의 내부 코너 개수 (가로 x 세로)                      |
| `--square 0.025` | 사각형 한 칸의 실제 크기 (미터 단위: 2.5cm이면 `0.025`)      |
| `--image_topic`  | 카메라 이미지 토픽 이름 (`/image_raw` 또는 `/camera/image_raw`) |
| `--camera_name`  | 저장될 보정 파일 이름의 prefix (`~/.ros/camera_info/{camera_name}.yaml`) |







---



[튜토리얼 목록](../README.md) 







