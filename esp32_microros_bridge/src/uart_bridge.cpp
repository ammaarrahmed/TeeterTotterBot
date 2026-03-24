#include "uart_bridge.h"

#include <Arduino.h>

#include <HardwareSerial.h>
#include <string.h>

#include "config.h"

namespace {

HardwareSerial robot_uart(ROBOT_UART_PORT_INDEX);

uint8_t frame_buffer[sizeof(TelemetryPacket)] = {};
size_t frame_index = 0;

TelemetryPacket latest_telemetry = {};
uint32_t latest_telemetry_ms = 0;
bool have_telemetry = false;

uint32_t rx_frame_count = 0;
uint32_t dropped_frame_count = 0;

void reset_parser()
{
	frame_index = 0;
}

void absorb_byte(const uint8_t byte)
{
	if (frame_index == 0) {
		if (byte == ROBOT_PROTOCOL_START_BYTE_1) {
			frame_buffer[frame_index++] = byte;
		}
		return;
	}

	if (frame_index == 1) {
		if (byte == ROBOT_PROTOCOL_TELEMETRY_START_BYTE_2) {
			frame_buffer[frame_index++] = byte;
		} else if (byte == ROBOT_PROTOCOL_START_BYTE_1) {
			frame_buffer[0] = byte;
			frame_index = 1;
		} else {
			reset_parser();
		}
		return;
	}

	frame_buffer[frame_index++] = byte;

	if (frame_index == sizeof(TelemetryPacket)) {
		TelemetryPacket candidate = {};
		memcpy(&candidate, frame_buffer, sizeof(candidate));

		if (robot_protocol_validate_telemetry(&candidate)) {
			latest_telemetry = candidate;
			latest_telemetry_ms = millis();
			have_telemetry = true;
			++rx_frame_count;
		} else {
			++dropped_frame_count;
		}

		reset_parser();
	}
}

}  // namespace

void uart_bridge_begin()
{
	robot_uart.begin(ROBOT_UART_BAUD, SERIAL_8N1, ROBOT_UART_RX_PIN, ROBOT_UART_TX_PIN);
	robot_uart.setRxBufferSize(512);
}

void uart_bridge_poll()
{
	while (robot_uart.available() > 0) {
		absorb_byte(static_cast<uint8_t>(robot_uart.read()));
	}
}

bool uart_bridge_copy_latest_telemetry(TelemetryPacket *telemetry, uint32_t *age_ms)
{
	if (!have_telemetry) {
		return false;
	}

	*telemetry = latest_telemetry;
	if (age_ms != nullptr) {
		*age_ms = millis() - latest_telemetry_ms;
	}

	return true;
}

void uart_bridge_send_control_packet(const ControlPacket *packet)
{
	ControlPacket frame = *packet;
	robot_protocol_prepare_control(&frame);
	robot_uart.write(reinterpret_cast<const uint8_t *>(&frame), sizeof(frame));
}

uint32_t uart_bridge_received_frames()
{
	return rx_frame_count;
}

uint32_t uart_bridge_dropped_frames()
{
	return dropped_frame_count;
}
