// User-editable runtime configuration for the ESP32 micro-ROS bridge.

#pragma once

#include <stdint.h>

static const char ROBOT_WIFI_SSID[] = "MICRO_ROS_AGENT";
static const char ROBOT_WIFI_PASSWORD[] = "11221122";

static constexpr uint8_t ROBOT_AGENT_IP[4] = {192,168 ,0 ,100 };
static constexpr uint16_t ROBOT_AGENT_PORT = 8888;

static constexpr uint32_t ROBOT_DEBUG_BAUD = 115200;
static constexpr uint32_t ROBOT_UART_BAUD = 115200;

static constexpr int ROBOT_UART_RX_PIN = 16;
static constexpr int ROBOT_UART_TX_PIN = 17;
static constexpr int ROBOT_UART_PORT_INDEX = 2;

static constexpr int ROBOT_STATUS_LED_PIN = 2;

static constexpr uint32_t ROBOT_CONTROL_SEND_PERIOD_MS = 20;
static constexpr uint32_t ROBOT_TELEMETRY_PUBLISH_PERIOD_MS = 20;
static constexpr uint32_t ROBOT_CMD_VEL_TIMEOUT_MS = 500;
static constexpr uint32_t ROBOT_AGENT_RECONNECT_INTERVAL_MS = 2000;
static constexpr uint32_t ROBOT_AGENT_PING_INTERVAL_MS = 1000;
static constexpr uint32_t ROBOT_AGENT_PING_TIMEOUT_MS = 100;
static constexpr uint8_t ROBOT_AGENT_PING_ATTEMPTS = 1;
static constexpr bool ROBOT_ENABLE_BALANCING_WHEN_CONNECTED = true;

static constexpr float ROBOT_MAX_LINEAR_MPS = 0.40f;
static constexpr float ROBOT_MAX_ANGULAR_RPS = 2.00f;
static constexpr float ROBOT_PITCH_TRIM_RAD = 0.0f;

static const char ROBOT_NODE_NAME[] = "teetertotter_bridge";
static const char ROBOT_NODE_NAMESPACE[] = "";
static const char ROBOT_CMD_VEL_TOPIC[] = "/cmd_vel";
static const char ROBOT_IMU_TOPIC[] = "/imu/data";
static const char ROBOT_BATTERY_TOPIC[] = "/battery_voltage";
static const char ROBOT_LEFT_COMMAND_TOPIC[] = "/left_motor_command";
static const char ROBOT_RIGHT_COMMAND_TOPIC[] = "/right_motor_command";
static const char ROBOT_STATUS_TOPIC[] = "/robot_status";
