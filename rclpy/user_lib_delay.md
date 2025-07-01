ROS 코드에서 `time.sleep()`함수의 사용은 권장되지 않는다. 사용하지 않는 것을 강력히 권고한다. ROS 2의 `rclpy`는 **이벤트 기반 프레임워크**이므로, 타이머 콜백, 서브스크립션 콜백 등은 `rclpy.spin()`이 실행되고 있어야 동작하지만, 

다음은 1초단위 딜레이 함수 `sec()`와 1`ms`단위 딜레이 함수 `ms()` 를 제공하는 사용자 정의 라이브러리 `delay.py`이다.  `time.sleep()`함수는 **파이썬 인터프리터 전체를 블로킹(blocking)** 하여 `rclpy.spin()`을 멈추게 하므로 사용하지 않는 것을 원칙으로 하며, 필요한 경우 `timer`를 사용한다. 다음 `delay.py`코드는 필요한 경우 `import`하여 ROS에서 `time.sleep()`과 같은 기능을 제공한다. 

```python
import rclpy
from rclpy.node import Node
from ar_track.delay import Delay

class Delay(Node):
       
    def __init__(self):
        super().__init__('delay_lib')
        periode = 0.001
        self.create_timer(0.001, self.timer_cb_1ms)
        self.create_timer(1.0, self.timer_cb_1s)
        
        self.count_sec = 0
        self.count_ms = 0
    
    def timer_cb_1ms(self):
        self.count_ms = self.count_ms + 1
    
    def ms(self,ms):
        duration = self.count_ms + ms
        
        while self.count_ms < duration:
            rclpy.spin_once(self, timeout_sec=0.001)
            pass
        self.count_ms = 0
    
    def timer_cb_1s(self):
        self.count_sec = self.count_sec + 1
    
    def sec(self,sec):
        duration = self.count_sec + sec
        
        while self.count_sec < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
            pass
        self.count_sec = 0

```

딜레이 기능이 필요한 패키지의 폴더에 작성하거나 복사 후 다음 예제의 3행과 같이 `import`하여 사용한다.

`from ar_track.delay import Delay` 이 문장에서 유추해 볼 때 `ar_track`이라는 패키지에서 딜레이 함수를 사용하려는 것으로 보이며, 그렇다면 `~/robot_ws/ar_track/ar_track`폴더에 `delay.py`파일이 존재 해야 하며,  `delay.py`파일은 `Delay`클라스를 가지고 있어야 한다.

```python
import rclpy
from rclpy.node import Node
from ar_track.delay import Delay #  <<--------

class TestDelay(Node):
      
    def __init__(self):
        
        super().__init__('test_delay')
 
def main(args=None):
    rclpy.init(args=args)
    node = TestDelay()
    delay = Delay()
    
    try:
        while rclpy.ok():
            node.get_logger().info("print this message every 1second!")
            #delay.ms(1000);
            delay.sec(1);
            
                
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt(SIGINT)')
        
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
            
if __name__ == '__main__':
    main()
```

