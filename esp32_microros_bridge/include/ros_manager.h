#pragma once

#include <stdint.h>

#include "robot_protocol.h"

void ros_manager_begin();
void ros_manager_tick();
void ros_manager_update_telemetry(const TelemetryPacket *telemetry);
void ros_manager_fill_control_packet(ControlPacket *packet);
bool ros_manager_is_connected();
const char *ros_manager_state_label();