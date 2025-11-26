#include "pid.h"

static inline float clamp_float(float value, float min_value, float max_value)
{
    if(value > max_value)
    {
        return max_value;
    }
    if(value < min_value)
    {
        return min_value;
    }
    return value;
}

void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float output_limit, float integral_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_limit = output_limit;
    pid->integral_limit = integral_limit;
    pid_reset(pid);
}

void pid_reset(pid_controller_t *pid)
{
    pid->integral = 0;
    pid->last_error = 0;
}

float pid_update(pid_controller_t *pid, float target, float measurement)
{
    float error = target - measurement;
    pid->integral += error;

    // 限制积分，避免积分饱和
    pid->integral = clamp_float(pid->integral, -pid->integral_limit, pid->integral_limit);

    float derivative = error - pid->last_error;
    pid->last_error = error;

    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    float limited_output = clamp_float(output, -pid->output_limit, pid->output_limit);

    // 如果输出被限幅且积分项仍在推向饱和方向，则回退积分，抑制积分饱和
    if(limited_output != output)
    {
        if((limited_output == pid->output_limit && error > 0) ||
           (limited_output == -pid->output_limit && error < 0))
        {
            pid->integral -= error;
            pid->integral = clamp_float(pid->integral, -pid->integral_limit, pid->integral_limit);
        }
        output = limited_output;
    }

    return output;
}
