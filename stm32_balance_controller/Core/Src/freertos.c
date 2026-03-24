#include "freertos_tasks.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "pid_controller.h"

typedef struct {
	float pitch_rad;
	float pitch_rate_rps;
	float battery_voltage_v;
	float target_linear_mps;
	float target_angular_rps;
	float pitch_trim_rad;
	uint32_t last_command_tick;
	uint32_t telemetry_sequence;
	int16_t left_motor_command;
	int16_t right_motor_command;
	uint8_t balancing_enabled;
	uint8_t imu_healthy;
	uint8_t status_flags;
} RobotRuntimeState;

static RobotRuntimeState robot_state;
static PidController balance_pid;
static uint8_t uart_rx_byte;
static uint8_t control_frame_buffer[sizeof(ControlPacket)];
static size_t control_frame_index = 0u;
static float gyro_bias_rps = 0.0f;
static uint32_t last_imu_tick = 0u;
static uint32_t last_control_tick = 0u;
static uint32_t last_telemetry_tick = 0u;
static volatile uint16_t software_pwm_left_compare = 0u;
static volatile uint16_t software_pwm_right_compare = 0u;
static volatile uint8_t software_pwm_phase = 0u;

static void robot_imu_task_step(uint32_t now_tick);
static void robot_control_task_step(uint32_t now_tick);
static void robot_telemetry_task_step(uint32_t now_tick);
static HAL_StatusTypeDef mpu6050_write_register(uint8_t reg, uint8_t value);
static HAL_StatusTypeDef mpu6050_read_registers(uint8_t start_reg, uint8_t *data, uint16_t length);
static HAL_StatusTypeDef mpu6050_initialize(void);
static HAL_StatusTypeDef mpu6050_calibrate_gyro(void);
static HAL_StatusTypeDef mpu6050_read_pitch(float *pitch_rad, float *pitch_rate_rps, float dt_seconds);
static float robot_read_battery_voltage(void);
static float robot_select_axis_value(uint8_t axis, float x_value, float y_value, float z_value);
static int16_t clamp_motor_command(float command);
static void robot_run_motor_self_test(uint32_t now_tick);
static void robot_apply_motor_command(int16_t left_command, int16_t right_command);
static void robot_stop_motors(void);
static void robot_process_control_packet(const ControlPacket *packet);
static void robot_process_uart_byte(uint8_t byte);
static void robot_update_status_led(void);
static uint8_t robot_convert_compare_to_software_pwm_steps(uint16_t compare_value);
static void robot_write_enable_pin_levels(GPIO_PinState left_state, GPIO_PinState right_state);

static float clamp_float(const float value, const float minimum_value, const float maximum_value)
{
	if (value < minimum_value) {
		return minimum_value;
	}

	if (value > maximum_value) {
		return maximum_value;
	}

	return value;
}

static float robot_select_axis_value(const uint8_t axis, const float x_value, const float y_value, const float z_value)
{
	switch (axis) {
	case ROBOT_AXIS_X:
		return x_value;
	case ROBOT_AXIS_Y:
		return y_value;
	case ROBOT_AXIS_Z:
	default:
		return z_value;
	}
}

static void robot_run_motor_self_test(const uint32_t now_tick)
{
	const uint32_t phase = (now_tick / ROBOT_MOTOR_SELF_TEST_STEP_MS) % 8u;
	int16_t left_motor = 0;
	int16_t right_motor = 0;

	switch (phase) {
	case 0u:
		left_motor = ROBOT_MOTOR_SELF_TEST_COMMAND;
		break;
	case 1u:
		left_motor = 0;
		right_motor = 0;
		break;
	case 2u:
		left_motor = -ROBOT_MOTOR_SELF_TEST_COMMAND;
		break;
	case 3u:
		left_motor = 0;
		right_motor = 0;
		break;
	case 4u:
		right_motor = ROBOT_MOTOR_SELF_TEST_COMMAND;
		break;
	case 5u:
		left_motor = 0;
		right_motor = 0;
		break;
	case 6u:
		right_motor = -ROBOT_MOTOR_SELF_TEST_COMMAND;
		break;
	case 7u:
	default:
		left_motor = 0;
		right_motor = 0;
		break;
	}

	robot_apply_motor_command(left_motor, right_motor);

	robot_state.left_motor_command = left_motor;
	robot_state.right_motor_command = right_motor;
	robot_state.status_flags = ROBOT_STATUS_BALANCING_ENABLED | ROBOT_STATUS_IMU_HEALTHY;
}

void MX_FREERTOS_Init(void)
{
	robot_runtime_init();
}

void robot_runtime_init(void)
{
	robot_stop_motors();

	memset(&robot_state, 0, sizeof(robot_state));
	pid_controller_init(
		&balance_pid,
		ROBOT_DEFAULT_PITCH_KP,
		ROBOT_DEFAULT_PITCH_KI,
		ROBOT_DEFAULT_PITCH_KD,
		-(float)ROBOT_MAX_MOTOR_COMMAND,
		(float)ROBOT_MAX_MOTOR_COMMAND,
		-0.35f,
		0.35f);
	last_imu_tick = HAL_GetTick();
	last_control_tick = last_imu_tick;
	last_telemetry_tick = last_imu_tick;

	if (mpu6050_initialize() != HAL_OK || mpu6050_calibrate_gyro() != HAL_OK) {
		robot_state.imu_healthy = 0u;
		robot_state.status_flags &= (uint8_t)~ROBOT_STATUS_IMU_HEALTHY;
	} else {
		robot_state.imu_healthy = 1u;
		robot_state.status_flags |= ROBOT_STATUS_IMU_HEALTHY;
	}
}

void robot_runtime_poll(void)
{
	const uint32_t now_tick = HAL_GetTick();

	if ((now_tick - last_imu_tick) >= ROBOT_IMU_PERIOD_MS) {
		last_imu_tick = now_tick;
		robot_imu_task_step(now_tick);
	}

	if ((now_tick - last_control_tick) >= ROBOT_CONTROL_PERIOD_MS) {
		last_control_tick = now_tick;
		robot_control_task_step(now_tick);
	}

	if ((now_tick - last_telemetry_tick) >= ROBOT_TELEMETRY_PERIOD_MS) {
		last_telemetry_tick = now_tick;
		robot_telemetry_task_step(now_tick);
	}
}

void robot_uart_start_receive_it(void)
{
	control_frame_index = 0u;
	(void)HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1u);
}

void robot_uart_rx_complete_callback(UART_HandleTypeDef *huart)
{
	if (huart != &huart1) {
		return;
	}

	robot_process_uart_byte(uart_rx_byte);
	(void)HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1u);
}

void robot_control_emergency_stop(void)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	robot_state.balancing_enabled = 0u;
	robot_state.left_motor_command = 0;
	robot_state.right_motor_command = 0;
	robot_state.status_flags |= ROBOT_STATUS_COMMAND_TIMEOUT;
	__set_PRIMASK(primask);
	pid_controller_reset(&balance_pid);
	robot_stop_motors();
}

void robot_motor_pwm_tick_callback(void)
{
	if (ROBOT_USE_SOFTWARE_PWM_ENABLE_PINS == 0u) {
		return;
	}

	const uint8_t left_steps = robot_convert_compare_to_software_pwm_steps(software_pwm_left_compare);
	const uint8_t right_steps = robot_convert_compare_to_software_pwm_steps(software_pwm_right_compare);
	const GPIO_PinState left_state = (left_steps > software_pwm_phase) ? GPIO_PIN_SET : GPIO_PIN_RESET;
	const GPIO_PinState right_state = (right_steps > software_pwm_phase) ? GPIO_PIN_SET : GPIO_PIN_RESET;

	robot_write_enable_pin_levels(left_state, right_state);

	software_pwm_phase++;
	if (software_pwm_phase >= ROBOT_SOFTWARE_PWM_STEPS) {
		software_pwm_phase = 0u;
	}
}

void robot_get_latest_telemetry(TelemetryPacket *packet)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	packet->sequence = robot_state.telemetry_sequence;
	packet->pitch_rad = robot_state.pitch_rad;
	packet->pitch_rate_rps = robot_state.pitch_rate_rps;
	packet->battery_voltage_v = robot_state.battery_voltage_v;
	packet->left_motor_command = robot_state.left_motor_command;
	packet->right_motor_command = robot_state.right_motor_command;
	packet->status_flags = robot_state.status_flags;
	__set_PRIMASK(primask);
	robot_protocol_prepare_telemetry(packet);
}

static void robot_imu_task_step(uint32_t now_tick)
{
	float pitch_rad = 0.0f;
	float pitch_rate_rps = 0.0f;
	(void)now_tick;

	if (mpu6050_read_pitch(&pitch_rad, &pitch_rate_rps, ROBOT_IMU_PERIOD_MS / 1000.0f) == HAL_OK) {
		uint32_t primask = __get_PRIMASK();
		__disable_irq();
		robot_state.pitch_rad = pitch_rad;
		robot_state.pitch_rate_rps = pitch_rate_rps;
		robot_state.imu_healthy = 1u;
		robot_state.status_flags |= ROBOT_STATUS_IMU_HEALTHY;
		__set_PRIMASK(primask);
	} else {
		uint32_t primask = __get_PRIMASK();
		__disable_irq();
		robot_state.imu_healthy = 0u;
		robot_state.status_flags &= (uint8_t)~ROBOT_STATUS_IMU_HEALTHY;
		__set_PRIMASK(primask);
		robot_stop_motors();
	}
}

static void robot_control_task_step(uint32_t now)
{
	if (ROBOT_ENABLE_MOTOR_SELF_TEST != 0u) {
		robot_run_motor_self_test(now);
		robot_update_status_led();
		return;
	}

	uint32_t primask = __get_PRIMASK();
	float pitch_rad;
	float target_linear_mps;
	float target_angular_rps;
	float pitch_trim_rad;
	uint8_t enable_balancing;
	uint8_t imu_healthy;
	uint32_t last_command_tick;

	__disable_irq();
	pitch_rad = robot_state.pitch_rad;
	target_linear_mps = robot_state.target_linear_mps;
	target_angular_rps = robot_state.target_angular_rps;
	pitch_trim_rad = robot_state.pitch_trim_rad;
	enable_balancing = robot_state.balancing_enabled;
	imu_healthy = robot_state.imu_healthy;
	last_command_tick = robot_state.last_command_tick;
	__set_PRIMASK(primask);

	if (!imu_healthy) {
		robot_control_emergency_stop();
		return;
	}

	if ((now - last_command_tick) > ROBOT_COMMAND_TIMEOUT_MS) {
		primask = __get_PRIMASK();
		__disable_irq();
		robot_state.status_flags |= ROBOT_STATUS_COMMAND_TIMEOUT;
		robot_state.balancing_enabled = 0u;
		__set_PRIMASK(primask);
		enable_balancing = 0u;
	}

	if (fabsf(pitch_rad) > ROBOT_MAX_SAFE_TILT_RAD) {
		primask = __get_PRIMASK();
		__disable_irq();
		robot_state.status_flags |= ROBOT_STATUS_TILT_CUTOFF;
		robot_state.balancing_enabled = 0u;
		__set_PRIMASK(primask);
		enable_balancing = 0u;
	} else {
		primask = __get_PRIMASK();
		__disable_irq();
		robot_state.status_flags &= (uint8_t)~ROBOT_STATUS_TILT_CUTOFF;
		__set_PRIMASK(primask);
	}

	if (enable_balancing != 0u) {
		const float pitch_target = pitch_trim_rad + (target_linear_mps * ROBOT_LINEAR_TO_PITCH_GAIN);
		const float balance_output = pid_controller_update(&balance_pid, pitch_target, pitch_rad, ROBOT_CONTROL_PERIOD_MS / 1000.0f);
		const float steering_output = target_angular_rps * ROBOT_ANGULAR_TO_STEERING_GAIN;
		const int16_t left_motor = clamp_motor_command(balance_output - steering_output);
		const int16_t right_motor = clamp_motor_command(balance_output + steering_output);

		robot_apply_motor_command(left_motor, right_motor);

		primask = __get_PRIMASK();
		__disable_irq();
		robot_state.left_motor_command = left_motor;
		robot_state.right_motor_command = right_motor;
		robot_state.status_flags &= (uint8_t)~ROBOT_STATUS_COMMAND_TIMEOUT;
		if (left_motor >= ROBOT_MAX_MOTOR_COMMAND || left_motor <= -ROBOT_MAX_MOTOR_COMMAND ||
			right_motor >= ROBOT_MAX_MOTOR_COMMAND || right_motor <= -ROBOT_MAX_MOTOR_COMMAND) {
			robot_state.status_flags |= ROBOT_STATUS_MOTOR_SATURATED;
		} else {
			robot_state.status_flags &= (uint8_t)~ROBOT_STATUS_MOTOR_SATURATED;
		}
		robot_state.status_flags |= ROBOT_STATUS_BALANCING_ENABLED;
		__set_PRIMASK(primask);
	} else {
		pid_controller_reset(&balance_pid);
		robot_stop_motors();
		primask = __get_PRIMASK();
		__disable_irq();
		robot_state.left_motor_command = 0;
		robot_state.right_motor_command = 0;
		robot_state.status_flags &= (uint8_t)~ROBOT_STATUS_BALANCING_ENABLED;
		__set_PRIMASK(primask);
	}

	robot_update_status_led();
}

static void robot_telemetry_task_step(uint32_t now_tick)
{
	TelemetryPacket telemetry = {0};
	(void)now_tick;

	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	robot_state.battery_voltage_v = robot_read_battery_voltage();
	robot_state.telemetry_sequence++;
	__set_PRIMASK(primask);

	robot_get_latest_telemetry(&telemetry);
	(void)HAL_UART_Transmit(&huart1, (uint8_t *)&telemetry, sizeof(telemetry), ROBOT_UART_TIMEOUT_MS);
}

static HAL_StatusTypeDef mpu6050_write_register(const uint8_t reg, const uint8_t value)
{
	return HAL_I2C_Mem_Write(&hi2c1, ROBOT_MPU6050_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&value, 1u, ROBOT_IMU_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef mpu6050_read_registers(const uint8_t start_reg, uint8_t *data, const uint16_t length)
{
	return HAL_I2C_Mem_Read(&hi2c1, ROBOT_MPU6050_I2C_ADDRESS, start_reg, I2C_MEMADD_SIZE_8BIT, data, length, ROBOT_IMU_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef mpu6050_initialize(void)
{
	uint8_t who_am_i = 0u;

	if (mpu6050_write_register(ROBOT_MPU6050_PWR_MGMT_1_REG, 0x00u) != HAL_OK) {
		return HAL_ERROR;
	}

	HAL_Delay(100);

	if (mpu6050_read_registers(ROBOT_MPU6050_WHO_AM_I_REG, &who_am_i, 1u) != HAL_OK) {
		return HAL_ERROR;
	}

	if (who_am_i != ROBOT_MPU6050_WHO_AM_I_EXPECTED_VALUE) {
		return HAL_ERROR;
	}

	if (mpu6050_write_register(ROBOT_MPU6050_SMPLRT_DIV_REG, 0x04u) != HAL_OK) {
		return HAL_ERROR;
	}

	if (mpu6050_write_register(ROBOT_MPU6050_CONFIG_REG, 0x03u) != HAL_OK) {
		return HAL_ERROR;
	}

	if (mpu6050_write_register(ROBOT_MPU6050_GYRO_CONFIG_REG, 0x00u) != HAL_OK) {
		return HAL_ERROR;
	}

	if (mpu6050_write_register(ROBOT_MPU6050_ACCEL_CONFIG_REG, 0x00u) != HAL_OK) {
		return HAL_ERROR;
	}

	return HAL_OK;
}

static HAL_StatusTypeDef mpu6050_calibrate_gyro(void)
{
	uint8_t raw_data[14] = {0};
	float accumulated_rate_rps = 0.0f;

	for (uint32_t sample = 0; sample < 500u; ++sample) {
		if (mpu6050_read_registers(ROBOT_MPU6050_ACCEL_XOUT_H_REG, raw_data, sizeof(raw_data)) != HAL_OK) {
			return HAL_ERROR;
		}

		const int16_t gyro_x_raw = (int16_t)((raw_data[8] << 8) | raw_data[9]);
		const int16_t gyro_y_raw = (int16_t)((raw_data[10] << 8) | raw_data[11]);
		const int16_t gyro_z_raw = (int16_t)((raw_data[12] << 8) | raw_data[13]);
		const float selected_gyro_raw = robot_select_axis_value(
			ROBOT_IMU_GYRO_PITCH_AXIS,
			(float)gyro_x_raw,
			(float)gyro_y_raw,
			(float)gyro_z_raw);
		accumulated_rate_rps += ((selected_gyro_raw / ROBOT_GYRO_SCALE_LSB_PER_DPS) * ROBOT_PI / 180.0f);
		HAL_Delay(2);
	}

	gyro_bias_rps = accumulated_rate_rps / 500.0f;
	return HAL_OK;
}

static HAL_StatusTypeDef mpu6050_read_pitch(float *pitch_rad, float *pitch_rate_rps, const float dt_seconds)
{
	static float filtered_pitch_rad = 0.0f;
	uint8_t raw_data[14] = {0};

	if (mpu6050_read_registers(ROBOT_MPU6050_ACCEL_XOUT_H_REG, raw_data, sizeof(raw_data)) != HAL_OK) {
		return HAL_ERROR;
	}

	const int16_t accel_x_raw = (int16_t)((raw_data[0] << 8) | raw_data[1]);
	const int16_t accel_y_raw = (int16_t)((raw_data[2] << 8) | raw_data[3]);
	const int16_t accel_z_raw = (int16_t)((raw_data[4] << 8) | raw_data[5]);
	const int16_t gyro_x_raw = (int16_t)((raw_data[8] << 8) | raw_data[9]);
	const int16_t gyro_y_raw = (int16_t)((raw_data[10] << 8) | raw_data[11]);
	const int16_t gyro_z_raw = (int16_t)((raw_data[12] << 8) | raw_data[13]);
	const float accel_x_g = (float)accel_x_raw / ROBOT_ACCEL_SCALE_LSB_PER_G;
	const float accel_y_g = (float)accel_y_raw / ROBOT_ACCEL_SCALE_LSB_PER_G;
	const float accel_z_g = (float)accel_z_raw / ROBOT_ACCEL_SCALE_LSB_PER_G;
	const float accel_pitch_component =
		robot_select_axis_value(ROBOT_IMU_ACCEL_PITCH_AXIS, accel_x_g, accel_y_g, accel_z_g) * ROBOT_IMU_ACCEL_PITCH_SIGN;
	const float accel_up_component =
		robot_select_axis_value(ROBOT_IMU_ACCEL_UP_AXIS, accel_x_g, accel_y_g, accel_z_g) * ROBOT_IMU_ACCEL_UP_SIGN;
	const float accel_pitch_rad = atan2f(accel_pitch_component, accel_up_component);
	const float gyro_pitch_raw = robot_select_axis_value(
		ROBOT_IMU_GYRO_PITCH_AXIS,
		(float)gyro_x_raw,
		(float)gyro_y_raw,
		(float)gyro_z_raw);
	const float gyro_rate_rps = (((gyro_pitch_raw / ROBOT_GYRO_SCALE_LSB_PER_DPS) * ROBOT_PI / 180.0f) - gyro_bias_rps) * ROBOT_IMU_GYRO_PITCH_SIGN;

	filtered_pitch_rad =
		(ROBOT_COMPLEMENTARY_ALPHA * (filtered_pitch_rad + (gyro_rate_rps * dt_seconds))) +
		((1.0f - ROBOT_COMPLEMENTARY_ALPHA) * accel_pitch_rad);

	*pitch_rad = filtered_pitch_rad;
	*pitch_rate_rps = gyro_rate_rps;
	return HAL_OK;
}

static float robot_read_battery_voltage(void)
{
	if (HAL_ADC_Start(&hadc1) != HAL_OK) {
		return 0.0f;
	}

	if (HAL_ADC_PollForConversion(&hadc1, 10u) != HAL_OK) {
		(void)HAL_ADC_Stop(&hadc1);
		return 0.0f;
	}

	const uint32_t raw_value = HAL_ADC_GetValue(&hadc1);
	(void)HAL_ADC_Stop(&hadc1);

	return (((float)raw_value / ROBOT_BATTERY_ADC_FULL_SCALE) * ROBOT_BATTERY_ADC_REFERENCE_V) * ROBOT_BATTERY_DIVIDER_RATIO;
}

static int16_t clamp_motor_command(const float command)
{
	float limited_command = clamp_float(command, -(float)ROBOT_MAX_MOTOR_COMMAND, (float)ROBOT_MAX_MOTOR_COMMAND);

	if (limited_command > 0.0f && limited_command < (float)ROBOT_MIN_EFFECTIVE_MOTOR_COMMAND) {
		limited_command = (float)ROBOT_MIN_EFFECTIVE_MOTOR_COMMAND;
	} else if (limited_command < 0.0f && limited_command > -(float)ROBOT_MIN_EFFECTIVE_MOTOR_COMMAND) {
		limited_command = -(float)ROBOT_MIN_EFFECTIVE_MOTOR_COMMAND;
	}

	return (int16_t)limited_command;
}

static uint8_t robot_convert_compare_to_software_pwm_steps(const uint16_t compare_value)
{
	if (compare_value == 0u) {
		return 0u;
	}

	uint32_t scaled_steps = ((uint32_t)compare_value * ROBOT_SOFTWARE_PWM_STEPS) / ROBOT_PWM_PERIOD_TICKS;
	if (scaled_steps == 0u) {
		scaled_steps = 1u;
	}
	if (scaled_steps > ROBOT_SOFTWARE_PWM_STEPS) {
		scaled_steps = ROBOT_SOFTWARE_PWM_STEPS;
	}

	return (uint8_t)scaled_steps;
}

static void robot_write_enable_pin_levels(const GPIO_PinState left_state, const GPIO_PinState right_state)
{
	HAL_GPIO_WritePin(ROBOT_LEFT_MOTOR_ENABLE_GPIO_Port, ROBOT_LEFT_MOTOR_ENABLE_Pin, left_state);
	HAL_GPIO_WritePin(ROBOT_RIGHT_MOTOR_ENABLE_GPIO_Port, ROBOT_RIGHT_MOTOR_ENABLE_Pin, right_state);
}

static void robot_apply_motor_command(const int16_t left_command, const int16_t right_command)
{
	const uint16_t left_pwm = (uint16_t)abs(left_command);
	const uint16_t right_pwm = (uint16_t)abs(right_command);

	if (left_command > 0) {
		HAL_GPIO_WritePin(ROBOT_LEFT_MOTOR_IN1_GPIO_Port, ROBOT_LEFT_MOTOR_IN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(ROBOT_LEFT_MOTOR_IN2_GPIO_Port, ROBOT_LEFT_MOTOR_IN2_Pin, GPIO_PIN_RESET);
	} else if (left_command < 0) {
		HAL_GPIO_WritePin(ROBOT_LEFT_MOTOR_IN1_GPIO_Port, ROBOT_LEFT_MOTOR_IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ROBOT_LEFT_MOTOR_IN2_GPIO_Port, ROBOT_LEFT_MOTOR_IN2_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(ROBOT_LEFT_MOTOR_IN1_GPIO_Port, ROBOT_LEFT_MOTOR_IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ROBOT_LEFT_MOTOR_IN2_GPIO_Port, ROBOT_LEFT_MOTOR_IN2_Pin, GPIO_PIN_RESET);
	}

	if (right_command > 0) {
		HAL_GPIO_WritePin(ROBOT_RIGHT_MOTOR_IN1_GPIO_Port, ROBOT_RIGHT_MOTOR_IN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(ROBOT_RIGHT_MOTOR_IN2_GPIO_Port, ROBOT_RIGHT_MOTOR_IN2_Pin, GPIO_PIN_RESET);
	} else if (right_command < 0) {
		HAL_GPIO_WritePin(ROBOT_RIGHT_MOTOR_IN1_GPIO_Port, ROBOT_RIGHT_MOTOR_IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ROBOT_RIGHT_MOTOR_IN2_GPIO_Port, ROBOT_RIGHT_MOTOR_IN2_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(ROBOT_RIGHT_MOTOR_IN1_GPIO_Port, ROBOT_RIGHT_MOTOR_IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ROBOT_RIGHT_MOTOR_IN2_GPIO_Port, ROBOT_RIGHT_MOTOR_IN2_Pin, GPIO_PIN_RESET);
	}

	if (ROBOT_USE_SOFTWARE_PWM_ENABLE_PINS != 0u) {
		uint32_t primask = __get_PRIMASK();
		__disable_irq();
		software_pwm_left_compare = left_pwm;
		software_pwm_right_compare = right_pwm;
		__set_PRIMASK(primask);
	} else {
		__HAL_TIM_SET_COMPARE(&ROBOT_LEFT_MOTOR_PWM_TIMER, ROBOT_LEFT_MOTOR_PWM_CHANNEL, left_pwm);
		__HAL_TIM_SET_COMPARE(&ROBOT_RIGHT_MOTOR_PWM_TIMER, ROBOT_RIGHT_MOTOR_PWM_CHANNEL, right_pwm);
	}
}

static void robot_stop_motors(void)
{
	HAL_GPIO_WritePin(ROBOT_LEFT_MOTOR_IN1_GPIO_Port, ROBOT_LEFT_MOTOR_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ROBOT_LEFT_MOTOR_IN2_GPIO_Port, ROBOT_LEFT_MOTOR_IN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ROBOT_RIGHT_MOTOR_IN1_GPIO_Port, ROBOT_RIGHT_MOTOR_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ROBOT_RIGHT_MOTOR_IN2_GPIO_Port, ROBOT_RIGHT_MOTOR_IN2_Pin, GPIO_PIN_RESET);
	if (ROBOT_USE_SOFTWARE_PWM_ENABLE_PINS != 0u) {
		uint32_t primask = __get_PRIMASK();
		__disable_irq();
		software_pwm_left_compare = 0u;
		software_pwm_right_compare = 0u;
		__set_PRIMASK(primask);
		robot_write_enable_pin_levels(GPIO_PIN_RESET, GPIO_PIN_RESET);
	} else {
		__HAL_TIM_SET_COMPARE(&ROBOT_LEFT_MOTOR_PWM_TIMER, ROBOT_LEFT_MOTOR_PWM_CHANNEL, 0u);
		__HAL_TIM_SET_COMPARE(&ROBOT_RIGHT_MOTOR_PWM_TIMER, ROBOT_RIGHT_MOTOR_PWM_CHANNEL, 0u);
	}
}

static void robot_process_control_packet(const ControlPacket *packet)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	robot_state.target_linear_mps = packet->target_linear_mps;
	robot_state.target_angular_rps = packet->target_angular_rps;
	robot_state.pitch_trim_rad = packet->pitch_trim_rad;
	robot_state.balancing_enabled = packet->enable_balancing;
	robot_state.last_command_tick = HAL_GetTick();
	robot_state.status_flags &= (uint8_t)~ROBOT_STATUS_COMMAND_TIMEOUT;
	__set_PRIMASK(primask);
}

static void robot_process_uart_byte(const uint8_t byte)
{
	if (control_frame_index == 0u) {
		if (byte != ROBOT_PROTOCOL_START_BYTE_1) {
			return;
		}
		control_frame_buffer[control_frame_index++] = byte;
		return;
	}

	if (control_frame_index == 1u) {
		if (byte != ROBOT_PROTOCOL_CONTROL_START_BYTE_2) {
			control_frame_index = 0u;
			if (byte == ROBOT_PROTOCOL_START_BYTE_1) {
				control_frame_buffer[control_frame_index++] = byte;
			}
			return;
		}
		control_frame_buffer[control_frame_index++] = byte;
		return;
	}

	control_frame_buffer[control_frame_index++] = byte;

	if (control_frame_index >= sizeof(ControlPacket)) {
		ControlPacket packet;
		memcpy(&packet, control_frame_buffer, sizeof(packet));
		control_frame_index = 0u;

		if (robot_protocol_validate_control(&packet) != 0u) {
			robot_process_control_packet(&packet);
		}
	}
}

static void robot_update_status_led(void)
{
	if ((robot_state.status_flags & ROBOT_STATUS_BALANCING_ENABLED) != 0u) {
		HAL_GPIO_WritePin(ROBOT_STATUS_LED_GPIO_Port, ROBOT_STATUS_LED_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(ROBOT_STATUS_LED_GPIO_Port, ROBOT_STATUS_LED_Pin, GPIO_PIN_SET);
	}
}
