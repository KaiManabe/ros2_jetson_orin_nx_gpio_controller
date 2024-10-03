/**
 * @file Main.cpp
 * @brief main関数の宣言
 */

#include "rclcpp/rclcpp.hpp"
#include "jetsonorinnx_gpio_controller_node/GpioControllerNode.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include <gpiod.h>

int main(int argc, char **argv){
    rclcpp::init(argc, argv);

    //ノードを宣言
    auto node = std::make_shared<GpioControllerNode>();

    //ノードを走らせる（無限ループ）
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}