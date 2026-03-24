#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	float kp;
	float ki;
	float kd;
	float integral;
	float previous_error;
	float output_min;
	float output_max;
	float integral_min;
	float integral_max;
	uint8_t has_previous_sample;
} PidController;

void pid_controller_init(
	PidController *controller,
	float kp,
	float ki,
	float kd,
	float output_min,
	float output_max,
	float integral_min,
	float integral_max);
void pid_controller_reset(PidController *controller);
float pid_controller_update(PidController *controller, float setpoint, float measurement, float dt_seconds);

#ifdef __cplusplus
}
#endif
