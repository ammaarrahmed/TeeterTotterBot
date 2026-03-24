#pragma once

#include <stdint.h>

#include "robot_protocol.h"

void uart_bridge_begin();
void uart_bridge_poll();
bool uart_bridge_copy_latest_telemetry(TelemetryPacket *telemetry, uint32_t *age_ms);
void uart_bridge_send_control_packet(const ControlPacket *packet);
uint32_t uart_bridge_received_frames();
uint32_t uart_bridge_dropped_frames();