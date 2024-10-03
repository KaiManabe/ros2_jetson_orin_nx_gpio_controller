# 概要
## src/jetsonorinnx_gpio_controller
- Jetson Orin NXのGPIOを制御するROS2ノード
- libgpiodを使用

### src/jetsonorinnx_gpio_controller/lineNumberGenerator
- ピンヘッダにおけるpin番号とline番号の変換をする配列・関数を生成する
- src/jetsonorinnx_gpio_controller/lineNumberGenerator/output/README.md 参照

## src/gpio_msgs
- コントローラと各ノード間でGPIOに関する指令をやりとりするためのカスタムメッセージ
- gpio_msgs/msg/GpioMode / gpio_msgs/msg/GpioSignal

## gpio_sample
- コントローラと通信するノードのサンプル
- rosGpio.pyをモジュールとして使用するとArduinoと同じメソッド名で記述可能
- rosGpio.pyの使用例→sample.py

<br><br>

# 前提条件
- Jetson Orin NXで動作を確認
    - Ubuntu 22.04.5 LTS
    - R36 (release), REVISION: 2.0, GCID: 35084178, BOARD: generic, EABI: aarch64
- libgpiodが必要
```bash
sudo apt install -y libgpiod2 libgpiod-dev libgpiod-doc gpiod
```

<br><br>
# 使用方法
1. このリポジトリをclone
```bash
git clone https://github.com/KaiManabe/ros2_jetson_orin_nx_gpio_controller.git
```

2. src/配下をワークスペース内src/配下にコピー
    - (もしくはリポジトリディレクトリ(ros2_jetson_orin_nx_gpio_controller)をwsとして使用)
```bash
cp -r ros2_jetson_orin_nx_gpio_controller/src/* {PATH_TO_YOUR_WS}/src/
```

3. ビルド
```bash
colcon build
source install/setup.bash
```

4. 実行
```bash
ros2 run jetsonOrinNX_gpio_controller jetsonOrinNX_gpio_controller
#↑別窓↓
ros2 run gpio_sample sample
```
