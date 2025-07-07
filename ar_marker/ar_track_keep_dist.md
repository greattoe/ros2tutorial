

---

## AR마커를 이용한 터틀봇3 제어 1. 거리유지(`keep_dist.py`)

**빌드 환경 :**  colcon **/** Ubuntu 20.04 **/** Foxy

`aruco_markers`토픽을 `subscribe`하여 `pose.pose.posion.z`값이 정해진 거리+마진보다 멀면 `/cmd_vel`토픽의 `linear.x`에 `+`값을 할당 후 토픽을 `publish`하여 터틀봇3를 전진 시키고,  `pose.pose.posion.z`값이 정해진-마진보다 가까우면 `/cmd_vel`토픽의 `linear.x`에 `-`값을 할당 후 토픽을 `publish`하여 터틀봇3를 후진 시키고, `pose.pose.posion.z`값이 `정해진 거리 - 마진` ~ `정해진 거리 + 마진`사이의 값인  경우 `/cmd_vel`토픽의 `linear.x`에 `0.0`을 할당 후, 토픽을 `publish`하여 터틀봇3를 멈추는 코드이다. 

---







```
ros2 run ar_track pub_tb3pose2d
```

위 명령 실행 시 다음과 같은 에러가 발생하면

```
from tf_transformations import euler_from_quaternion#, quaternion_from_euler
ModuleNotFoundError: No module named 'tf_transformations'
```



다음을 실행한다.

```
sudo apt install python3-tf-transformations
```





`ros2_aruco`패키지의 노드 실행 시 다음과 같은 에러가 발생한다면,



```bash
ros2 run ros2_aruco aruco_raspicam2 
Traceback (most recent call last):
  File "/home/gnd0/robot_ws/install/ros2_aruco/lib/ros2_aruco/aruco_raspicam2", line 11, in <module>
    load_entry_point('ros2-aruco', 'console_scripts', 'aruco_raspicam2')()
  File "/home/gnd0/robot_ws/build/ros2_aruco/ros2_aruco/aruco_raspicam2.py", line 165, in main
    node = ArucoNode()
  File "/home/gnd0/robot_ws/build/ros2_aruco/ros2_aruco/aruco_raspicam2.py", line 91, in __init__
    self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
AttributeError: module 'cv2.aruco' has no attribute 'Dictionary_get'
```



그 원인은OpenCV의 **`aruco` 모듈이 설치되지 않았거나**, OpenCV 버전이 너무 낮아서 `cv2.aruco.Dictionary_get()` 함수가 없기 때문이다.



일반 `opencv-python` 패키지에는 `aruco` 서브모듈이 없으므로  `opencv-contrib-python`을 설치해야 한다.



기존의 `opencv`삭제

```
pip3 uninstall opencv-python
```



```
pip3 uninstall opencv-python opencv-contrib-python opencv-python-headless -y
```





 `opencv-contrib-python` 설치 (ArUco 포함)



```
pip3 install opencv-contrib-python
```



```
pip3 install opencv-contrib-python-headless
```

설치

```
pip3 install opencv-contrib-python==4.5.5.64
```

설치 확인

```
python3 -c "import cv2; print(cv2.__version__); print(hasattr(cv2.aruco, 'Dictionary_get'))"
```

```
4.5.5
True
```







```
ros2 run opencv img_compressed2raw
```



```
 ros2 run ros2_aruco aruco_raspicam2
```







[튜토리얼 목록](../README.md) 







