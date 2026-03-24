#include <Arduino.h>

#include "config.h"
#include "ros_manager.h"
#include "uart_bridge.h"

namespace {

uint32_t last_control_sent_ms = 0;
uint32_t last_status_log_ms = 0;
bool led_state = false;

void log_startup_banner()
{
  Serial.println();
  Serial.println("TeeterTotter Bot ESP32 Bridge");
  Serial.println("----------------------------------------");
  Serial.printf("UART port: %d | RX: GPIO%d | TX: GPIO%d | baud: %lu\n",
          ROBOT_UART_PORT_INDEX,
          ROBOT_UART_RX_PIN,
          ROBOT_UART_TX_PIN,
          static_cast<unsigned long>(ROBOT_UART_BAUD));
  Serial.printf("Agent: %u.%u.%u.%u:%u\n",
          ROBOT_AGENT_IP[0],
          ROBOT_AGENT_IP[1],
          ROBOT_AGENT_IP[2],
          ROBOT_AGENT_IP[3],
          ROBOT_AGENT_PORT);
  Serial.printf("Wi-Fi SSID: %s\n", ROBOT_WIFI_SSID);
  if (strcmp(ROBOT_WIFI_SSID, "REPLACE_WITH_WIFI_SSID") == 0) {
    Serial.println("WARNING: update ROBOT_WIFI_SSID and ROBOT_WIFI_PASSWORD in include/config.h");
  }
}

void update_status_led()
{
  static uint32_t last_toggle_ms = 0;
  const uint32_t now_ms = millis();
  const uint32_t blink_period_ms = ros_manager_is_connected() ? 500 : 100;

  if (now_ms - last_toggle_ms >= blink_period_ms) {
    last_toggle_ms = now_ms;
    led_state = !led_state;
    digitalWrite(ROBOT_STATUS_LED_PIN, led_state ? HIGH : LOW);
  }
}

void publish_status_log()
{
  const uint32_t now_ms = millis();

  if (now_ms - last_status_log_ms < 5000) {
    return;
  }

  last_status_log_ms = now_ms;

  TelemetryPacket telemetry = {};
  uint32_t telemetry_age_ms = 0;
  const bool have_telemetry = uart_bridge_copy_latest_telemetry(&telemetry, &telemetry_age_ms);

  Serial.printf("ROS state: %s | UART frames: %lu ok / %lu dropped",
          ros_manager_state_label(),
          static_cast<unsigned long>(uart_bridge_received_frames()),
          static_cast<unsigned long>(uart_bridge_dropped_frames()));

  if (have_telemetry) {
    Serial.printf(" | pitch: %.3f rad | battery: %.2f V | age: %lu ms",
            telemetry.pitch_rad,
            telemetry.battery_voltage_v,
            static_cast<unsigned long>(telemetry_age_ms));
  }

  Serial.println();
}

}  // namespace

void setup()
{
  pinMode(ROBOT_STATUS_LED_PIN, OUTPUT);
  digitalWrite(ROBOT_STATUS_LED_PIN, LOW);

  Serial.begin(ROBOT_DEBUG_BAUD);
  delay(1500);

  log_startup_banner();
  uart_bridge_begin();
  ros_manager_begin();
}

void loop()
{
  uart_bridge_poll();

  TelemetryPacket telemetry = {};
  uint32_t telemetry_age_ms = 0;
  if (uart_bridge_copy_latest_telemetry(&telemetry, &telemetry_age_ms)) {
    ros_manager_update_telemetry(&telemetry);
  }

  ros_manager_tick();

  const uint32_t now_ms = millis();
  if (now_ms - last_control_sent_ms >= ROBOT_CONTROL_SEND_PERIOD_MS) {
    ControlPacket control = {};
    ros_manager_fill_control_packet(&control);
    uart_bridge_send_control_packet(&control);
    last_control_sent_ms = now_ms;
  }

  update_status_led();
  publish_status_log();
  delay(1);
}