#pragma once

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim2;
extern ADC_HandleTypeDef hadc1;

#define ROBOT_MPU6050_ADDRESS_PIN_LOW 0u
#define ROBOT_MPU6050_ADDRESS_PIN_HIGH 1u
#define ROBOT_MPU6050_ADDRESS_PIN_STATE ROBOT_MPU6050_ADDRESS_PIN_LOW
#define ROBOT_MPU6050_I2C_ADDRESS ((((ROBOT_MPU6050_ADDRESS_PIN_STATE) == ROBOT_MPU6050_ADDRESS_PIN_HIGH) ? 0x69u : 0x68u) << 1)
#define ROBOT_MPU6050_PWR_MGMT_1_REG 0x6Bu
#define ROBOT_MPU6050_SMPLRT_DIV_REG 0x19u
#define ROBOT_MPU6050_CONFIG_REG 0x1Au
#define ROBOT_MPU6050_GYRO_CONFIG_REG 0x1Bu
#define ROBOT_MPU6050_ACCEL_CONFIG_REG 0x1Cu
#define ROBOT_MPU6050_ACCEL_XOUT_H_REG 0x3Bu
#define ROBOT_MPU6050_WHO_AM_I_REG 0x75u
#define ROBOT_MPU6050_WHO_AM_I_EXPECTED_VALUE (((ROBOT_MPU6050_ADDRESS_PIN_STATE) == ROBOT_MPU6050_ADDRESS_PIN_HIGH) ? 0x69u : 0x68u)

#define ROBOT_AXIS_X 0u
#define ROBOT_AXIS_Y 1u
#define ROBOT_AXIS_Z 2u

#define ROBOT_CONTROL_PERIOD_MS 5u
#define ROBOT_IMU_PERIOD_MS 5u
#define ROBOT_TELEMETRY_PERIOD_MS 20u
#define ROBOT_COMMAND_TIMEOUT_MS 250u
#define ROBOT_UART_TIMEOUT_MS 10u
#define ROBOT_IMU_I2C_TIMEOUT_MS 20u
#define ROBOT_MAX_SAFE_TILT_RAD 0.55f
#define ROBOT_COMPLEMENTARY_ALPHA 0.985f
#define ROBOT_GYRO_SCALE_LSB_PER_DPS 131.0f
#define ROBOT_ACCEL_SCALE_LSB_PER_G 16384.0f
#define ROBOT_GRAVITY_MPS2 9.80665f
#define ROBOT_BATTERY_ADC_REFERENCE_V 3.3f
#define ROBOT_BATTERY_ADC_FULL_SCALE 4095.0f
#define ROBOT_BATTERY_DIVIDER_RATIO 3.0f
#define ROBOT_PI 3.14159265358979323846f
#define ROBOT_PWM_TIMER_PRESCALER 419u
#define ROBOT_PWM_PERIOD_TICKS 1000u
#define ROBOT_USE_SOFTWARE_PWM_ENABLE_PINS 1u
#define ROBOT_SOFTWARE_PWM_TIMER_PRESCALER 83u
#define ROBOT_SOFTWARE_PWM_TIMER_PERIOD_TICKS 99u
#define ROBOT_SOFTWARE_PWM_STEPS 100u
#define ROBOT_MAX_MOTOR_COMMAND 950
#define ROBOT_MIN_EFFECTIVE_MOTOR_COMMAND 180
#define ROBOT_ENABLE_MOTOR_SELF_TEST 0u
#define ROBOT_MOTOR_SELF_TEST_COMMAND ROBOT_MAX_MOTOR_COMMAND
#define ROBOT_MOTOR_SELF_TEST_STEP_MS 1200u
#define ROBOT_FORCE_ENABLE_PINS_HIGH_FOR_TEST 0u
#define ROBOT_USE_ALT_ENABLE_PWM_PINS 0u
#define ROBOT_LINEAR_TO_PITCH_GAIN 0.22f
#define ROBOT_ANGULAR_TO_STEERING_GAIN 180.0f
#define ROBOT_DEFAULT_PITCH_KP 32.0f
#define ROBOT_DEFAULT_PITCH_KI 3.0f
#define ROBOT_DEFAULT_PITCH_KD 0.85f

/*
 * Bench motor test defaults.
 *
 * These settings keep the proven PB0/PB1 wiring and use software PWM on those
 * same pins for normal robot operation. GPIO-high enable already proved the
 * wiring is correct, so this mode bypasses the non-working timer AF PWM path.
 *
 * Baseline wiring for this diagnostic:
 * - PB0 -> ENA
 * - PB1 -> ENB
 * - PB12/PB13 -> left direction
 * - PB14/PB15 -> right direction
 */

/*
 * MPU6050 mounting configuration.
 *
 * The defaults below assume:
 * - accelerometer X points in the robot pitch direction
 * - accelerometer Z points upward when the robot is upright
 * - gyro Y measures pitch rate
 *
 * If /robot_status reports tilt cutoff while the robot is near upright,
 * or /imu/data shows a pitch near +/-90 degrees at rest, these axis choices
 * do not match your physical MPU6050 mounting and should be adjusted.
 */
#define ROBOT_IMU_ACCEL_PITCH_AXIS ROBOT_AXIS_Y
#define ROBOT_IMU_ACCEL_UP_AXIS ROBOT_AXIS_Z
#define ROBOT_IMU_GYRO_PITCH_AXIS ROBOT_AXIS_X
#define ROBOT_IMU_ACCEL_PITCH_SIGN 1.0f
#define ROBOT_IMU_ACCEL_UP_SIGN 1.0f
#define ROBOT_IMU_GYRO_PITCH_SIGN 1.0f

#if ROBOT_USE_ALT_ENABLE_PWM_PINS
#define ROBOT_LEFT_MOTOR_PWM_TIMER htim2
#define ROBOT_RIGHT_MOTOR_PWM_TIMER htim2
#define ROBOT_LEFT_MOTOR_PWM_CHANNEL TIM_CHANNEL_1
#define ROBOT_RIGHT_MOTOR_PWM_CHANNEL TIM_CHANNEL_2
#define ROBOT_LEFT_MOTOR_ENABLE_GPIO_Port GPIOA
#define ROBOT_LEFT_MOTOR_ENABLE_Pin GPIO_PIN_0
#define ROBOT_RIGHT_MOTOR_ENABLE_GPIO_Port GPIOA
#define ROBOT_RIGHT_MOTOR_ENABLE_Pin GPIO_PIN_1
#else
#define ROBOT_LEFT_MOTOR_PWM_TIMER htim3
#define ROBOT_RIGHT_MOTOR_PWM_TIMER htim3
#define ROBOT_LEFT_MOTOR_PWM_CHANNEL TIM_CHANNEL_3
#define ROBOT_RIGHT_MOTOR_PWM_CHANNEL TIM_CHANNEL_4
#define ROBOT_LEFT_MOTOR_ENABLE_GPIO_Port GPIOB
#define ROBOT_LEFT_MOTOR_ENABLE_Pin GPIO_PIN_0
#define ROBOT_RIGHT_MOTOR_ENABLE_GPIO_Port GPIOB
#define ROBOT_RIGHT_MOTOR_ENABLE_Pin GPIO_PIN_1
#endif

#define ROBOT_LEFT_MOTOR_IN1_GPIO_Port GPIOB
#define ROBOT_LEFT_MOTOR_IN1_Pin GPIO_PIN_12
#define ROBOT_LEFT_MOTOR_IN2_GPIO_Port GPIOB
#define ROBOT_LEFT_MOTOR_IN2_Pin GPIO_PIN_13
#define ROBOT_RIGHT_MOTOR_IN1_GPIO_Port GPIOB
#define ROBOT_RIGHT_MOTOR_IN1_Pin GPIO_PIN_14
#define ROBOT_RIGHT_MOTOR_IN2_GPIO_Port GPIOB
#define ROBOT_RIGHT_MOTOR_IN2_Pin GPIO_PIN_15

#define ROBOT_STATUS_LED_GPIO_Port GPIOC
#define ROBOT_STATUS_LED_Pin GPIO_PIN_13

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_I2C1_Init(void);
void MX_USART1_UART_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_ADC1_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void Error_Handler(void);

#ifdef __cplusplus
}
#endif
