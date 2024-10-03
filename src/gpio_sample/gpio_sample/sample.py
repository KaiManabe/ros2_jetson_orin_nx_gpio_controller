import time
import rclpy
from rclpy.node import Node
from gpio_msgs.msg import GpioMode
from gpio_msgs.msg import GpioSignal
from rclpy.executors import MultiThreadedExecutor
from .rosGpio import rosGpio

class myNode(rosGpio):
    """rosGPIOを継承したROSノード
    通常のROSノードに加え、GPIOコントローラとの通信メソッドを備える
    """
    def __init__(self):
        """コンストラクタ
        スーパークラスのコンストラクタ引数にノード名を与える
        """
        
        super().__init__("sample_package_node_name")
        
        #ピンモードを設定
        self.setup()
        
        #以下Lチカ用
        self.state = False
        self.timer = self.create_timer(0.5, self.Lchika)
        

    def setup(self):
        self.pinMode(12, self.INPUT)
        self.pinMode(31, self.OUTPUT)
        self.pinMode(35, self.OUTPUT)
        
        self.attachCallback(12, self.myCallback)
        
    
    def Lchika(self):
        """31ピンはLチカする
        35ピンは12ピンの入力をコピーする
        """
        self.state = not(self.state)
        self.digitalWrite(31, self.state)
        self.digitalWrite(35, self.digitalRead(12))


    def myCallback(self, msg):
        self.get_logger().info(f"Original callback called. Pin.{msg.number} is now {msg.signal}.")
    
    
def main():
    rclpy.init()
    node = myNode()
    
    #コールバック関数を並列処理するために必要
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        
        
if __name__ == '__main__':
    main()