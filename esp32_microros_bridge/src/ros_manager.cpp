#include "ros_manager.h"

#include <Arduino.h>
#include <WiFi.h>

#include <geometry_msgs/msg/twist.h>
#include <micro_ros_platformio.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>

#include <math.h>
#include <string.h>

#include "config.h"

namespace {

enum class RosConnectionState {
	kDisconnected,
	kConnected,
};

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t telemetry_timer;
rcl_subscription_t cmd_vel_subscription;
rcl_publisher_t imu_publisher;
rcl_publisher_t battery_publisher;
rcl_publisher_t left_motor_publisher;
rcl_publisher_t right_motor_publisher;
rcl_publisher_t status_publisher;
rclc_executor_t executor;

geometry_msgs__msg__Twist cmd_vel_message;
sensor_msgs__msg__Imu imu_message;
std_msgs__msg__Float32 battery_message;
std_msgs__msg__Int32 left_motor_message;
std_msgs__msg__Int32 right_motor_message;
std_msgs__msg__Int32 status_message;

TelemetryPacket latest_telemetry = {};
bool have_telemetry = false;
uint32_t last_telemetry_ms = 0;

float latest_linear_mps = 0.0f;
float latest_angular_rps = 0.0f;
uint32_t last_cmd_vel_ms = 0;
uint32_t control_sequence = 0;

uint32_t last_reconnect_attempt_ms = 0;
uint32_t last_agent_ping_ms = 0;
RosConnectionState connection_state = RosConnectionState::kDisconnected;

#define RCCHECK(fn)                                  \
	do {                                             \
		rcl_ret_t rc = (fn);                         \
		if (rc != RCL_RET_OK) {                      \
			Serial.printf("RCL error: %d\n", rc);  \
			return false;                            \
		}                                            \
	} while (0)

#define RCSOFTCHECK(fn)                              \
	do {                                             \
		rcl_ret_t rc = (fn);                         \
		if (rc != RCL_RET_OK) {                      \
			Serial.printf("RCL soft error: %d\n", rc); \
		}                                            \
	} while (0)

void log_fini_result(const char *label, const rcl_ret_t rc)
{
	if (rc != RCL_RET_OK) {
		Serial.printf("Cleanup error in %s: %d\n", label, rc);
	}
}

float clamp_float(const float value, const float minimum_value, const float maximum_value)
{
	if (value < minimum_value) {
		return minimum_value;
	}

	if (value > maximum_value) {
		return maximum_value;
	}

	return value;
}

void fill_imu_message_from_telemetry()
{
	const float half_pitch = latest_telemetry.pitch_rad * 0.5f;
	const int64_t epoch_nanos = rmw_uros_epoch_nanos();

	if (epoch_nanos > 0) {
		imu_message.header.stamp.sec = static_cast<int32_t>(epoch_nanos / 1000000000LL);
		imu_message.header.stamp.nanosec = static_cast<uint32_t>(epoch_nanos % 1000000000LL);
	} else {
		imu_message.header.stamp.sec = static_cast<int32_t>(millis() / 1000UL);
		imu_message.header.stamp.nanosec = static_cast<uint32_t>((millis() % 1000UL) * 1000000UL);
	}

	imu_message.orientation.x = 0.0;
	imu_message.orientation.y = sinf(half_pitch);
	imu_message.orientation.z = 0.0;
	imu_message.orientation.w = cosf(half_pitch);

	imu_message.angular_velocity.x = 0.0;
	imu_message.angular_velocity.y = latest_telemetry.pitch_rate_rps;
	imu_message.angular_velocity.z = latest_telemetry.right_motor_command - latest_telemetry.left_motor_command;

	imu_message.linear_acceleration_covariance[0] = -1.0;

	battery_message.data = latest_telemetry.battery_voltage_v;
	left_motor_message.data = latest_telemetry.left_motor_command;
	right_motor_message.data = latest_telemetry.right_motor_command;
	status_message.data = latest_telemetry.status_flags;
}

void cmd_vel_callback(const void *message)
{
	const auto *twist = static_cast<const geometry_msgs__msg__Twist *>(message);

	latest_linear_mps = clamp_float(static_cast<float>(twist->linear.x), -ROBOT_MAX_LINEAR_MPS, ROBOT_MAX_LINEAR_MPS);
	latest_angular_rps = clamp_float(static_cast<float>(twist->angular.z), -ROBOT_MAX_ANGULAR_RPS, ROBOT_MAX_ANGULAR_RPS);
	last_cmd_vel_ms = millis();
}

void telemetry_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
	(void)last_call_time;
	if (timer == nullptr || !have_telemetry) {
		return;
	}

	fill_imu_message_from_telemetry();

	RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_message, nullptr));
	RCSOFTCHECK(rcl_publish(&battery_publisher, &battery_message, nullptr));
	RCSOFTCHECK(rcl_publish(&left_motor_publisher, &left_motor_message, nullptr));
	RCSOFTCHECK(rcl_publish(&right_motor_publisher, &right_motor_message, nullptr));
	RCSOFTCHECK(rcl_publish(&status_publisher, &status_message, nullptr));
}

bool create_entities()
{
	allocator = rcl_get_default_allocator();

	memset(&support, 0, sizeof(support));
	node = rcl_get_zero_initialized_node();
	telemetry_timer = rcl_get_zero_initialized_timer();
	cmd_vel_subscription = rcl_get_zero_initialized_subscription();
	imu_publisher = rcl_get_zero_initialized_publisher();
	battery_publisher = rcl_get_zero_initialized_publisher();
	left_motor_publisher = rcl_get_zero_initialized_publisher();
	right_motor_publisher = rcl_get_zero_initialized_publisher();
	status_publisher = rcl_get_zero_initialized_publisher();
	executor = rclc_executor_get_zero_initialized_executor();

	RCCHECK(rclc_support_init(&support, 0, nullptr, &allocator));
	RCCHECK(rclc_node_init_default(&node, ROBOT_NODE_NAME, ROBOT_NODE_NAMESPACE, &support));

	RCCHECK(rclc_publisher_init_default(
		&imu_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
		ROBOT_IMU_TOPIC));

	RCCHECK(rclc_publisher_init_default(
		&battery_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		ROBOT_BATTERY_TOPIC));

	RCCHECK(rclc_publisher_init_default(
		&left_motor_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		ROBOT_LEFT_COMMAND_TOPIC));

	RCCHECK(rclc_publisher_init_default(
		&right_motor_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		ROBOT_RIGHT_COMMAND_TOPIC));

	RCCHECK(rclc_publisher_init_default(
		&status_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		ROBOT_STATUS_TOPIC));

	RCCHECK(rclc_subscription_init_default(
		&cmd_vel_subscription,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		ROBOT_CMD_VEL_TOPIC));

	RCCHECK(rclc_timer_init_default2(
		&telemetry_timer,
		&support,
		RCL_MS_TO_NS(ROBOT_TELEMETRY_PUBLISH_PERIOD_MS),
		telemetry_timer_callback,
		true));

	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscription, &cmd_vel_message, &cmd_vel_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_timer(&executor, &telemetry_timer));

	rmw_uros_sync_session(1000);
	Serial.println("micro-ROS entities created");
	return true;
}

void destroy_entities()
{
	log_fini_result("imu_publisher", rcl_publisher_fini(&imu_publisher, &node));
	log_fini_result("battery_publisher", rcl_publisher_fini(&battery_publisher, &node));
	log_fini_result("left_motor_publisher", rcl_publisher_fini(&left_motor_publisher, &node));
	log_fini_result("right_motor_publisher", rcl_publisher_fini(&right_motor_publisher, &node));
	log_fini_result("status_publisher", rcl_publisher_fini(&status_publisher, &node));
	log_fini_result("cmd_vel_subscription", rcl_subscription_fini(&cmd_vel_subscription, &node));
	log_fini_result("telemetry_timer", rcl_timer_fini(&telemetry_timer));
	log_fini_result("executor", rclc_executor_fini(&executor));
	log_fini_result("node", rcl_node_fini(&node));
	log_fini_result("support", rclc_support_fini(&support));
	memset(&support, 0, sizeof(support));
	executor = rclc_executor_get_zero_initialized_executor();
	node = rcl_get_zero_initialized_node();
	telemetry_timer = rcl_get_zero_initialized_timer();
	cmd_vel_subscription = rcl_get_zero_initialized_subscription();
	imu_publisher = rcl_get_zero_initialized_publisher();
	battery_publisher = rcl_get_zero_initialized_publisher();
	left_motor_publisher = rcl_get_zero_initialized_publisher();
	right_motor_publisher = rcl_get_zero_initialized_publisher();
	status_publisher = rcl_get_zero_initialized_publisher();
	connection_state = RosConnectionState::kDisconnected;
}

void attempt_connection()
{
	const uint32_t now_ms = millis();
	if (now_ms - last_reconnect_attempt_ms < ROBOT_AGENT_RECONNECT_INTERVAL_MS) {
		return;
	}

	last_reconnect_attempt_ms = now_ms;
	Serial.println("Attempting micro-ROS agent ping...");

	if (rmw_uros_ping_agent(ROBOT_AGENT_PING_TIMEOUT_MS, ROBOT_AGENT_PING_ATTEMPTS) == RMW_RET_OK) {
		if (create_entities()) {
			connection_state = RosConnectionState::kConnected;
			last_agent_ping_ms = now_ms;
		}
	}
}

void monitor_connection_health()
{
	const uint32_t now_ms = millis();
	if (now_ms - last_agent_ping_ms < ROBOT_AGENT_PING_INTERVAL_MS) {
		return;
	}

	last_agent_ping_ms = now_ms;

	if (rmw_uros_ping_agent(ROBOT_AGENT_PING_TIMEOUT_MS, ROBOT_AGENT_PING_ATTEMPTS) != RMW_RET_OK) {
		Serial.println("micro-ROS agent lost, tearing down entities");
		destroy_entities();
	}
}

}  // namespace

void ros_manager_begin()
{
	IPAddress agent_ip(ROBOT_AGENT_IP[0], ROBOT_AGENT_IP[1], ROBOT_AGENT_IP[2], ROBOT_AGENT_IP[3]);
	set_microros_wifi_transports(const_cast<char *>(ROBOT_WIFI_SSID), const_cast<char *>(ROBOT_WIFI_PASSWORD), agent_ip, ROBOT_AGENT_PORT);

	memset(&cmd_vel_message, 0, sizeof(cmd_vel_message));
	memset(&imu_message, 0, sizeof(imu_message));
	memset(&battery_message, 0, sizeof(battery_message));
	memset(&left_motor_message, 0, sizeof(left_motor_message));
	memset(&right_motor_message, 0, sizeof(right_motor_message));
	memset(&status_message, 0, sizeof(status_message));

	imu_message.orientation_covariance[0] = 0.02;
	imu_message.orientation_covariance[4] = 0.02;
	imu_message.orientation_covariance[8] = 0.02;
	imu_message.angular_velocity_covariance[0] = 0.05;
	imu_message.angular_velocity_covariance[4] = 0.05;
	imu_message.angular_velocity_covariance[8] = 0.05;
	imu_message.linear_acceleration_covariance[0] = -1.0;

	last_cmd_vel_ms = millis();
	Serial.println("micro-ROS transport configured for Wi-Fi");
}

void ros_manager_tick()
{
	if (connection_state == RosConnectionState::kDisconnected) {
		attempt_connection();
		return;
	}

	monitor_connection_health();

	if (connection_state == RosConnectionState::kConnected) {
		RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2)));
	}
}

void ros_manager_update_telemetry(const TelemetryPacket *telemetry)
{
	latest_telemetry = *telemetry;
	have_telemetry = true;
	last_telemetry_ms = millis();
}

void ros_manager_fill_control_packet(ControlPacket *packet)
{
	const uint32_t now_ms = millis();
	const bool command_is_fresh = (now_ms - last_cmd_vel_ms) <= ROBOT_CMD_VEL_TIMEOUT_MS;
	const bool bridge_connected = ros_manager_is_connected();

	memset(packet, 0, sizeof(*packet));
	packet->sequence = ++control_sequence;
	packet->target_linear_mps = command_is_fresh ? latest_linear_mps : 0.0f;
	packet->target_angular_rps = command_is_fresh ? latest_angular_rps : 0.0f;
	packet->pitch_trim_rad = ROBOT_PITCH_TRIM_RAD;
	packet->enable_balancing = (bridge_connected && ROBOT_ENABLE_BALANCING_WHEN_CONNECTED) ? 1u : 0u;
	packet->reserved = 0u;
	robot_protocol_prepare_control(packet);
}

bool ros_manager_is_connected()
{
	return connection_state == RosConnectionState::kConnected;
}

const char *ros_manager_state_label()
{
	return ros_manager_is_connected() ? "connected" : "disconnected";
}
