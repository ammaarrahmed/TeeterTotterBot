#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ROBOT_PROTOCOL_VERSION 1u
#define ROBOT_PROTOCOL_START_BYTE_1 0xAAu
#define ROBOT_PROTOCOL_CONTROL_START_BYTE_2 0x55u
#define ROBOT_PROTOCOL_TELEMETRY_START_BYTE_2 0xEEu

#define ROBOT_STATUS_BALANCING_ENABLED 0x01u
#define ROBOT_STATUS_IMU_HEALTHY 0x02u
#define ROBOT_STATUS_COMMAND_TIMEOUT 0x04u
#define ROBOT_STATUS_TILT_CUTOFF 0x08u
#define ROBOT_STATUS_MOTOR_SATURATED 0x10u

#pragma pack(push, 1)
typedef struct {
	uint8_t start_byte_1;
	uint8_t start_byte_2;
	uint8_t version;
	uint32_t sequence;
	float target_linear_mps;
	float target_angular_rps;
	float pitch_trim_rad;
	uint8_t enable_balancing;
	uint8_t reserved;
	uint8_t checksum;
} ControlPacket;

typedef struct {
	uint8_t start_byte_1;
	uint8_t start_byte_2;
	uint8_t version;
	uint32_t sequence;
	float pitch_rad;
	float pitch_rate_rps;
	float battery_voltage_v;
	int16_t left_motor_command;
	int16_t right_motor_command;
	uint8_t status_flags;
	uint8_t checksum;
} TelemetryPacket;
#pragma pack(pop)

static inline uint8_t robot_protocol_calculate_checksum(const void *data, size_t length)
{
	const uint8_t *bytes = (const uint8_t *)data;
	uint8_t checksum = 0;

	for (size_t index = 0; index < length; ++index) {
		checksum ^= bytes[index];
	}

	return checksum;
}

static inline void robot_protocol_prepare_control(ControlPacket *packet)
{
	packet->start_byte_1 = ROBOT_PROTOCOL_START_BYTE_1;
	packet->start_byte_2 = ROBOT_PROTOCOL_CONTROL_START_BYTE_2;
	packet->version = ROBOT_PROTOCOL_VERSION;
	packet->checksum = robot_protocol_calculate_checksum(packet, sizeof(ControlPacket) - 1u);
}

static inline void robot_protocol_prepare_telemetry(TelemetryPacket *packet)
{
	packet->start_byte_1 = ROBOT_PROTOCOL_START_BYTE_1;
	packet->start_byte_2 = ROBOT_PROTOCOL_TELEMETRY_START_BYTE_2;
	packet->version = ROBOT_PROTOCOL_VERSION;
	packet->checksum = robot_protocol_calculate_checksum(packet, sizeof(TelemetryPacket) - 1u);
}

static inline uint8_t robot_protocol_validate_control(const ControlPacket *packet)
{
	return packet->start_byte_1 == ROBOT_PROTOCOL_START_BYTE_1 &&
		   packet->start_byte_2 == ROBOT_PROTOCOL_CONTROL_START_BYTE_2 &&
		   packet->version == ROBOT_PROTOCOL_VERSION &&
		   packet->checksum == robot_protocol_calculate_checksum(packet, sizeof(ControlPacket) - 1u);
}

static inline uint8_t robot_protocol_validate_telemetry(const TelemetryPacket *packet)
{
	return packet->start_byte_1 == ROBOT_PROTOCOL_START_BYTE_1 &&
		   packet->start_byte_2 == ROBOT_PROTOCOL_TELEMETRY_START_BYTE_2 &&
		   packet->version == ROBOT_PROTOCOL_VERSION &&
		   packet->checksum == robot_protocol_calculate_checksum(packet, sizeof(TelemetryPacket) - 1u);
}

#ifdef __cplusplus
}
#endif
