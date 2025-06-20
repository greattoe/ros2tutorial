

---

## AR마커를 이용한 터틀봇3 제어 1. 거리유지(`keep_dist.py`)

**빌드 환경 :**  colcon **/** Ubuntu 20.04 **/** Foxy

`aruco_markers`토픽을 `subscribe`하여 `pose.pose.posion.z`값이 정해진 거리+마진보다 멀면 `/cmd_vel`토픽의 `linear.x`에 `+`값을 할당 후 토픽을 `publish`하여 터틀봇3를 전진 시키고,  `pose.pose.posion.z`값이 정해진-마진보다 가까우면 `/cmd_vel`토픽의 `linear.x`에 `-`값을 할당 후 토픽을 `publish`하여 터틀봇3를 후진 시키고, `pose.pose.posion.z`값이 `정해진 거리 - 마진` ~ `정해진 거리 + 마진`사이의 값인  경우 `/cmd_vel`토픽의 `linear.x`에 `0.0`을 할당 후, 토픽을 `publish`하여 터틀봇3를 멈추는 코드이다. 

---



[튜토리얼 목록](../README.md) 







