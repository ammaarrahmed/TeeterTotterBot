#include "pid_controller.h"

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

void pid_controller_init(
	PidController *controller,
	const float kp,
	const float ki,
	const float kd,
	const float output_min,
	const float output_max,
	const float integral_min,
	const float integral_max)
{
	controller->kp = kp;
	controller->ki = ki;
	controller->kd = kd;
	controller->output_min = output_min;
	controller->output_max = output_max;
	controller->integral_min = integral_min;
	controller->integral_max = integral_max;
	pid_controller_reset(controller);
}

void pid_controller_reset(PidController *controller)
{
	controller->integral = 0.0f;
	controller->previous_error = 0.0f;
	controller->has_previous_sample = 0u;
}

float pid_controller_update(PidController *controller, const float setpoint, const float measurement, const float dt_seconds)
{
	const float error = setpoint - measurement;
	float derivative = 0.0f;

	if (dt_seconds > 0.0f) {
		controller->integral += error * dt_seconds;
		controller->integral = clamp_float(controller->integral, controller->integral_min, controller->integral_max);
	}

	if (controller->has_previous_sample != 0u && dt_seconds > 0.0f) {
		derivative = (error - controller->previous_error) / dt_seconds;
	} else {
		controller->has_previous_sample = 1u;
	}

	controller->previous_error = error;

	return clamp_float(
		(controller->kp * error) + (controller->ki * controller->integral) + (controller->kd * derivative),
		controller->output_min,
		controller->output_max);
}
