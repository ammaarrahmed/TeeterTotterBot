# TeeterTotter Bot

Small self-balancing robot with an STM32 doing the fast control loop and an ESP32 doing the Wi-Fi and micro-ROS side.

It is still a bench project and still needs tuning, but the repo is in a usable state now.

## Parts Used

- STM32F401 Black Pill
- ESP32 dev board
- MPU6050
- L298N
- 2 N20 motors
- 2x 18650 cells
- XL4015 buck converter
- ST-Link V2

## Wiring

STM32 to MPU6050:

- PB6 -> SCL
- PB7 -> SDA
- GND -> GND
- 3.3V -> VCC
- GND -> ADD
- XDA, XCL, INT not used right now

STM32 to ESP32 UART:

- PA9 -> ESP32 GPIO16
- PA10 -> ESP32 GPIO17
- GND -> GND

STM32 to L298N:

- PB0 -> ENA
- PB1 -> ENB
- PB12 -> IN1
- PB13 -> IN2
- PB14 -> IN3
- PB15 -> IN4

Power idea used here:

- battery -> L298N motor input
- battery -> XL4015 input
- XL4015 5V output -> STM32 and ESP32 power
- all grounds common

## Things To Change In Code

ESP32 config is in `esp32_microros_bridge/include/config.h`.

Change these before flashing:

- `ROBOT_WIFI_SSID`
- `ROBOT_WIFI_PASSWORD`
- `ROBOT_AGENT_IP`
- `ROBOT_AGENT_PORT`
- `ROBOT_UART_RX_PIN`
- `ROBOT_UART_TX_PIN`
- topic names if you want different ones

STM32 tuning stuff is in `stm32_balance_controller/Core/Inc/main.h`.

Things you may end up changing there:

- PID gains
- pitch trim
- IMU axis/sign settings
- motor limits
- timeout values

## PC Commands

Use 2 terminals.

Terminal 1, micro-ROS agent:

```bash
source /opt/ros/jazzy/setup.bash
source ~/uros_host_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```

Terminal 2, ROS commands:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
```

If you want an ESP32 serial monitor too, run it in another terminal from `esp32_microros_bridge` with `pio device monitor`.

## Useful ROS Commands

List topics:

```bash
ros2 topic list
```

Echo IMU:

```bash
ros2 topic echo /imu/data
```

Echo status:

```bash
ros2 topic echo /robot_status
```

Echo battery:

```bash
ros2 topic echo /battery_voltage
```

Echo motor commands:

```bash
ros2 topic echo /left_motor_command
ros2 topic echo /right_motor_command
```

Small forward command:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.02}, angular: {z: 0.0}}" -r 10
```

Small reverse command:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.02}, angular: {z: 0.0}}" -r 10
```

Small turn command:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.2}}" -r 10
```

Stop command:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" -r 10
```

## Notes

First tests should be with wheels off the ground.

If it moves the wrong way, do not keep increasing gains. Fix sign, wiring, or IMU orientation first.

That is basically it for now.
