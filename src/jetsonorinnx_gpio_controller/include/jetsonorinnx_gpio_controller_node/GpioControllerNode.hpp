/**
 * @file GpioControllerNode.hpp
 * @brief GpioCOntrollerNodeクラスおよびそれが用いる構造体・列挙体・定数の定義
 */

#ifndef GPIOCONTROLLERNODE_HPP__

#define GPIOCONTROLLERNODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "gpio_msgs/msg/gpio_signal.hpp"
#include "gpio_msgs/msg/gpio_mode.hpp"
#include <gpiod.h>
#include <thread>

/*##############################################
定数宣言
##############################################*/
//GPIOが持つピン数
const int PIN_COUNT = 40;


/*##############################################
構造体・列挙対宣言
##############################################*/
//入出力方向を定義するもの
enum PINMODE{
    INPUT,
    OUTPUT,
    UNDEFINED
};

typedef struct _pin{
    enum PINMODE mode = UNDEFINED;
    struct gpiod_line *line;
} pin;



/*##############################################
クラス定義
##############################################*/
class GpioControllerNode : public rclcpp::Node{
    public:
        GpioControllerNode();
        ~GpioControllerNode();
    private:
        enum PINMODE getPinMode(int pinNum);
        void setPinMode(int pinNum, enum PINMODE mode);
        void output(int pinNum, bool value);
        bool input(int pinNum);
        int pinToLine(int pinNum); 

        void subCallbackPinMode(const gpio_msgs::msg::GpioMode msg);
        void subCallbackPinSignal(const gpio_msgs::msg::GpioSignal msg);
        void createPublisher(int pinNum);

        void pinMonitor();

        pin _pinStatus[PIN_COUNT + 1];
        struct gpiod_chip *_chip;
        rclcpp::Subscription<gpio_msgs::msg::GpioMode>::SharedPtr _subPinMode;
        rclcpp::Subscription<gpio_msgs::msg::GpioSignal>::SharedPtr _subPinSignal;
        rclcpp::Publisher<gpio_msgs::msg::GpioSignal>::SharedPtr _pubPinSignal[PIN_COUNT + 1];

        rclcpp::TimerBase::SharedPtr timer_;
        std::thread threadMonitor;
        bool running = true;
};


#endif