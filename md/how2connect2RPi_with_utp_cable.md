### 라즈베리파이 - 우분투PC 다이렉트 유선 연결







#### 우분투 PC

`settings`의 `Network` 열기

```
sudo ip addr add 192.168.10.1/24 dev <인터페이스명>
```



```
sudo ip link set <인터페이스명> up
```





```
sudo ip link set eth0 up
```





```
sudo ip addr add 192.168.10.2/24 dev eth0
```



```
sudo ip addr add 192.168.10.2/24 dev eth0
sudo ip link set eth0 up
sudo ip route add default via 192.168.10.1
```

