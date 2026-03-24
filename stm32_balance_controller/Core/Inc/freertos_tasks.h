#pragma once

#include "main.h"
#include "robot_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

void MX_FREERTOS_Init(void);
void robot_runtime_init(void);
void robot_runtime_poll(void);
void robot_uart_start_receive_it(void);
void robot_uart_rx_complete_callback(UART_HandleTypeDef *huart);
void robot_control_emergency_stop(void);
void robot_get_latest_telemetry(TelemetryPacket *packet);
void robot_motor_pwm_tick_callback(void);

#ifdef __cplusplus
}
#endif
