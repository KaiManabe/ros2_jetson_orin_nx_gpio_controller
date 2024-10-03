from rclpy.node import Node
from gpio_msgs.msg import GpioMode
from gpio_msgs.msg import GpioSignal
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy



class rosGpio(Node):
    """通常のROSノードに、GPIOコントローラと通信する機能を付加したもの
    通常のrosノード(rclpy.node.Node)ではなくこのクラスを継承することで使用可能

    Args:
        Node (str): ノード名
    """
    def __init__(self, nodeName: str):
        super().__init__(nodeName)
        
        """
        enumみたいにして使う定数たち
        """
        self.OUTPUT = GpioMode.OUTPUT
        self.INPUT = GpioMode.INPUT
        self.HIGH = True
        self.LOW = False
        
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        #pinMode/output用パブリッシャ
        self.pubGpioMode = self.create_publisher(GpioMode, "/gpio/mode", qos_profile)
        self.pubGpioOutput = self.create_publisher(GpioSignal, "/gpio/output", qos_profile)
        
        
        """
        入力モードになっているpin用オブジェクト
        {
            1:
                {
                    "subscriber" = subscriber,
                    "signal" = True/False,
                    "callback" = コールバック関数/None
                }
            12:
                {
                    ...
        }
        """
        self.pinObj = {}
    
    
    

    def pinMode(self, pin: int, mode: int):
        """GPIOピンの入出力方向を設定する

        Args:
            pin (int): 設定するピン番号
            mode (self.INPUT/self.OUTPUT): 入出力方向
        """
        msg = GpioMode()
        msg.number = pin
        msg.mode = mode
        self.pubGpioMode.publish(msg)
        
        #INPUTモードに設定された場合subscriberを作成し、pinObjにappendして管理
        if mode == self.INPUT:
            self.pinObj[pin] = {}
            self.pinObj[pin]["subscriber"] = self.create_subscription(GpioSignal,
                                                                        f"/gpio/input/pin{pin}",
                                                                        self.subCallback,
                                                                        10
                                                                        )
            self.pinObj[msg.number]["signal"] = False
            self.pinObj[msg.number]["callback"] = None
        
    
    def attachCallback(self, pin: int, func: function):
        """INPUTモードとしたピンの入力が変わるたびに実行されるコールバックを割り当てる
            既にpinMode()が実行されたpinに対してのみ実行可能
        Args:
            pin (int): ピン番号
            func (function): コールバック関数(一つの引数GpioSignalをとる)
        """
        self.pinObj[pin]["callback"] = func
    
    
    def subCallback(self, msg: GpioSignal):
        """ROS用コールバック関数
            外部からの呼び出しは無用

        Args:
            msg (GpioSignal): GpioSignalメッセージ
        """
        if self.pinObj[msg.number]["callback"] != None:
            self.pinObj[msg.number]["callback"](msg)
        
        self.pinObj[msg.number]["signal"] = msg.signal
    
    
    def digitalRead(self, pin: int):
        """GPIOに入力される信号を読む

        Args:
            pin (int): ピン番号

        Returns:
            bool: HIGH/LOW が True/Falseとして返ってくる
        """
        return self.pinObj[pin]["signal"]

    
    def digitalWrite(self, pin: int, signal: bool):
        """GPIOに信号を出力

        Args:
            pin (int): ピン番号
            signal (bool): HIGH/LOW を True/Falseとして指定
        """
        msg = GpioSignal()
        msg.number = pin
        msg.signal = signal
        self.pubGpioOutput.publish(msg)
