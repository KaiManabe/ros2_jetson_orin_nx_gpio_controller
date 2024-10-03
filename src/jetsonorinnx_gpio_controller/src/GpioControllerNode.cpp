/**
 * @file GpioControllerNode.cpp
 * @brief GpioControllerNodeの実態を宣言（メソッドなど内部処理の記述）
 */

#include "rclcpp/rclcpp.hpp"
#include "jetsonorinnx_gpio_controller_node/GpioControllerNode.hpp"
#include "gpio_msgs/msg/gpio_signal.hpp"
#include "gpio_msgs/msg/gpio_mode.hpp"
#include <gpiod.h>
#include <thread>

#include <chrono>
using namespace std::chrono_literals;  // リテラルを有効化する

/*
threadライブラリで並列処理することを許可する場合にdefine
/gpio/ * トピックに対して応答しなかったりする場合、おそらくthreadのせいなのでコメントアウトしてみる
    →その場合timerコールバックになるのでGPIO入力の応答が10ms以上遅くなる
*/
//#define GPIO_MONITOR_USING_THREAD

/*デバッグログ表示*/
//#define LOGGING_DEBUG

/*通常ログ表示*/
#define LOGGING_INFO



/**
 * @brief コンストラクタ - デバッグ用として、pin.31にHIGH/LOWを繰り返し出力するようになっている
 */
GpioControllerNode::GpioControllerNode() : Node("gpio_controller_node"){
    //使用するGPIO chipを開く
    this->_chip = gpiod_chip_open_by_name("gpiochip0");

    rclcpp::QoS qos = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    //pinmodeを変更する指令(/gpio/modeトピック)を受け取るsubscriber
    _subPinMode = this->create_subscription<gpio_msgs::msg::GpioMode>(
                "/gpio/mode", qos,
                std::bind(&GpioControllerNode::subCallbackPinMode, this, std::placeholders::_1)
                );
    
    
    //出力指令(/gpio/output)を受け取るsubscriber
    _subPinSignal = this->create_subscription<gpio_msgs::msg::GpioSignal>(
                "/gpio/output", qos,
                std::bind(&GpioControllerNode::subCallbackPinSignal, this, std::placeholders::_1)
                );

    //inputモードに指定されたpinに対する入力を監視する→別スレッドで回り続ける
#ifdef GPIO_MONITOR_USING_THREAD
    this->threadMonitor = std::thread(&GpioControllerNode::pinMonitor, this);
#else
    timer_ = this->create_wall_timer(10ms, [this]() -> void { this->pinMonitor(); });
#endif
    
    RCLCPP_INFO(this->get_logger(), "Initialized node.");
};


/**
 * @brief デストラクタ
 * 
 */
GpioControllerNode::~GpioControllerNode(){
#ifdef LOGGING_INFO
    RCLCPP_INFO(this->get_logger(), "Waiting for GPIO monitor thread.");
#endif
    //モニタ用スレッド終了待ち

#ifdef GPIO_MONITOR_USING_THREAD
    this->running = false;
    this->threadMonitor.join();
#endif

#ifdef LOGGING_INFO
    RCLCPP_INFO(this->get_logger(), "GPIO monitor thread joined.");
    RCLCPP_INFO(this->get_logger(), "Releasing all pins.");
#endif

    //すべてのpinの出力は0にしておく
    for(int i = 0; i <= PIN_COUNT; i++){
        if(this->getPinMode(i) == OUTPUT) this->output(i, false);
    }

#ifdef LOGGING_INFO
    RCLCPP_INFO(this->get_logger(), "Closing GPIO chip.");
#endif

    //チップの予約を解放
    gpiod_chip_close(this->_chip);
};

/**
 * @brief pinの入出力モードを取得
 * 
 * @param pinNum pin番号 (line番号ではない)
 * @return enum PINMODE INPUT/OUTPUT/UNDEFINEDのどれか
 */
enum PINMODE GpioControllerNode::getPinMode(int pinNum){
    return this->_pinStatus[pinNum].mode;
};


/**
 * @brief pinの入出力モードを設定
 * 
 * @param pinNum pin番号 (line番号ではない)
 * @param mode enum PINMODE INPUT/OUTPUT/UNDEFINEDのどれか
 */
void GpioControllerNode::setPinMode(int pinNum, enum PINMODE mode){
    if(this->_pinStatus[pinNum].mode == UNDEFINED){
        //入出力モードが定義されていない場合、pinを予約してline構造体を初期化
        this->_pinStatus[pinNum].line = gpiod_chip_get_line(this->_chip, pinToLine(pinNum));
    }

    this->_pinStatus[pinNum].mode = mode;

    //モードに応じて割り当てる
    //UNDEFINEDを割り当てられた場合、pinを解放する    
    if(mode == OUTPUT){
        gpiod_line_request_output(this->_pinStatus[pinNum].line, "gpio_controller_node", 0);
#ifdef LOGGING_INFO
        RCLCPP_INFO(this->get_logger(), "Pin.%d mode is set to OUTPUT.", pinNum);
#endif
    }else if(mode == INPUT){
        gpiod_line_request_input(this->_pinStatus[pinNum].line, "gpio_controller_node");
#ifdef LOGGING_INFO
        RCLCPP_INFO(this->get_logger(), "Pin.%d mode is set to INPUT.", pinNum);
#endif
    }else{
        gpiod_line_release(this->_pinStatus[pinNum].line);     
#ifdef LOGGING_INFO
        RCLCPP_INFO(this->get_logger(), "Released pin.%d.", pinNum);   
#endif
    }
};

/**
 * @brief GPIOから信号を出力する
 * 
 * @param pinNum pin番号 (line番号ではない)
 * @param value true/false 出力信号
 */
void GpioControllerNode::output(int pinNum, bool value){
    if(this->getPinMode(pinNum) != OUTPUT){
        RCLCPP_ERROR(this->get_logger(), "Pin.%d : Failed to exec output. Its current pin mode is not OUTPUT.", pinNum);
        return;
    }
    gpiod_line_set_value(this->_pinStatus[pinNum].line, static_cast<int>(value));
}

/**
 * @brief GPIOへの入力を取得する
 * 
 * @param pinNum pin番号 (line番号ではない)
 * @return bool 入力信号
 */
bool GpioControllerNode::input(int pinNum){
    if(this->getPinMode(pinNum) != INPUT){
        RCLCPP_ERROR(this->get_logger(), "Pin.%d : Failed to exec input. Its current pin mode is not INPUT.", pinNum);
        return false;
    } 
    return static_cast<bool>(gpiod_line_get_value(this->_pinStatus[pinNum].line));
}

/**
 * @brief pin番号をline番号に変換する
 * 
 * @param pinNum pin番号 (line番号ではない)
 * @return int line番号
 */
int GpioControllerNode::pinToLine(int pinNum){
    const int TMP[] = {-1,-1,-1,22,-1,21,-1,144,110,-1,111,112,50,122,-1,85,126,-1,136,135,-1,134,123,133,136,-1,137,20,19,105,-1,106,41,43,-1,53,113,124,52,-1,51};
    return TMP[pinNum];
}



/**
 * @brief /gpio/mode トピックのsubscriberのコールバック　メッセージに基づいてGPIOの入出力モードを設定
 * 
 * @param msg 
 */
void GpioControllerNode::subCallbackPinMode(const gpio_msgs::msg::GpioMode msg){
    int pin = msg.number;
    int mode = msg.mode;
#ifdef LOGGING_DEBUG
    RCLCPP_INFO(this->get_logger(), "[DEBUGLOG] pinMode received: %d, %d", pin, mode);    //DEBUG
#endif

    if(mode == gpio_msgs::msg::GpioMode::INPUT && this->getPinMode(pin) != INPUT){
        this->setPinMode(pin, INPUT);
        this->createPublisher(pin); //pin専用のpublisherを作成し割り当てる
    }else if(mode == gpio_msgs::msg::GpioMode::OUTPUT){
        this->setPinMode(pin, OUTPUT);
    }
}

/**
 * @brief /gpio/output トピックのsubscriberのコールバック　メッセージに基づいてGPIOの出力電圧を設定
 * 
 * @param msg 
 */
void GpioControllerNode::subCallbackPinSignal(const gpio_msgs::msg::GpioSignal msg){
    int pin = msg.number;
    bool signal = msg.signal;
#ifdef LOGGING_DEBUG
    RCLCPP_INFO(this->get_logger(), "[DEBUGLOG] pinSignal received: %d, %d", pin, (int)signal);    //DEBUG
#endif

    this->output(pin, signal);
}


/**
 * @brief GPIOピンを入力モードに設定→　ピンごとにトピックをわけpublisherを割り当てる→　初期状態をpublishする
 * 
 * @param pinNum 
 */
void GpioControllerNode::createPublisher(int pinNum){
    std::string topicName = "/gpio/input/pin" + std::to_string(pinNum);
    rclcpp::QoS qos = rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    this->_pubPinSignal[pinNum] = this->create_publisher<gpio_msgs::msg::GpioSignal>(topicName, qos);

    std::unique_ptr<gpio_msgs::msg::GpioSignal> msg = std::make_unique<gpio_msgs::msg::GpioSignal>();
    msg->number = pinNum;
    msg->signal = this->input(pinNum);
    this->_pubPinSignal[pinNum]->publish(std::move(msg));
    
}



/**
 * @brief 入力モードに割り当てられたGPIOピンを常に監視する　別スレッドで並列実行
 * 
 */
void GpioControllerNode::pinMonitor(){
    static bool currentState[PIN_COUNT + 1] = {false,};

#ifdef GPIO_MONITOR_USING_THREAD
    while(this->running){
#endif
        for(int i = 0; i <= PIN_COUNT; i++){
            if(this->_pubPinSignal[i]){
                bool tmp = this->input(i);
                if(currentState[i] != tmp){
                    std::unique_ptr<gpio_msgs::msg::GpioSignal> msg = std::make_unique<gpio_msgs::msg::GpioSignal>();
                    msg->number = i;
                    msg->signal = tmp;
                    this->_pubPinSignal[i]->publish(std::move(msg));
#ifdef LOGGING_DEBUG
                    RCLCPP_INFO(this->get_logger(), "[DEBUGLOG] publishing: %d, %d", i, (int)tmp);    //DEBUG
#endif
                    currentState[i] = tmp;
                }
            }
        }
#ifdef GPIO_MONITOR_USING_THREAD
    }
#endif
}