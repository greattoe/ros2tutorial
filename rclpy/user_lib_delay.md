다음은 1초단위 딜레이 함수 `sec()`와 1ms단위 딜레이 함수 `ms()`를 제공하는 사용자 정의 라이브러리 `delay.py`이다.

```python
import rclpy
from rclpy.node import Node

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

딜레이 기능이 필요한 패키지의 폴더에 작성하거나 복사 후 다음 예제와 같이 `import`하여 사용한다.

`test_delay.py`

```python
import rclpy
from rclpy.node import Node
from ar_track.delay import Delay


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

