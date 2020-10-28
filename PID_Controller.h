#pragma once
class PID_Controller
{

public:

	PID_Controller(float p, float i, float d) {
		p_gain = p;
		i_gain = i;
		d_gain = d;
		last_read_value = 0;
		accumulated_error = 0;
		error_limit = 1;
		output_limit_max = 850000;
		output_limit_min = 400000;

	}
	float pid_execute(float ref_value, float read_value);
	
private:
	float p_gain;
	float i_gain;
	float d_gain;
	float last_read_value;
	float accumulated_error;
	float error_limit;
	float output_limit_max;
	float output_limit_min;
};

