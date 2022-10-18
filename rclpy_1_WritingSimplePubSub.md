## rospy_tutorial/ Tutorials/ WritingPublisherSubscriber



---

## 퍼블리셔와 서브스크라이버 작성 

**출처 :**  <https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#create-a-package>

**튜토리얼 레벨 :**  초급

**빌드 환경 :**  colcon **/** Ubuntu 20.04 **/** Foxy

---

작업 경로를 워크스페이스`~/robot_ws` 의 `src` 폴더로 변경한다. 

```
cd ~/robot_ws/src
```

`rclpy` 와 `std_msgs` 에 의존성을 갖는 패키지 `mypkg` 생성

```
ros2 pkg create mypkg --build-type ament_python --dependencies rclpy std_msgs
```















[튜토리얼 목록](../README.md) 







