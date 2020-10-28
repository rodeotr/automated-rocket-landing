#include "PID_Controller.h"

float PID_Controller::pid_execute(float ref_value, float read_value) {
	float error = ref_value - read_value;
	accumulated_error += error;
	if (accumulated_error > error_limit) {
		accumulated_error = error_limit;
	}
	else if (accumulated_error < -error_limit) {
		accumulated_error = -error_limit;
	}
	float p_value = p_gain * error;
	float i_value = i_gain * accumulated_error;
	float d_value = d_gain * (last_read_value - read_value);

	last_read_value = read_value;
	float pid_output = p_value + i_value + d_value;
	if (output_limit_max < pid_output) {
		pid_output = output_limit_max;
	}
	else if (output_limit_min > pid_output) {
		pid_output = output_limit_min;
	}
	return pid_output;
}