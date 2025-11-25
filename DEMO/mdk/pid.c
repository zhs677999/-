#include "pid.h"

// ???PID??
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float max_out, float max_int) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_output = max_out;
    pid->max_integral = max_int;
    pid->error = 0;
    pid->last_error = 0;
    pid->prev_error = 0;
    pid->integral = 0;
    pid->output = 0;
}

// ??? PID ?? (??????????)
// ???? PWM ? ?? (???)
float PID_Calc_Incremental(PID_TypeDef *pid, float target, float feedback) {
    pid->error = target - feedback;
    
    float p_out = pid->kp * (pid->error - pid->last_error);
    float i_out = pid->ki * pid->error;
    float d_out = pid->kd * (pid->error - 2 * pid->last_error + pid->prev_error);
    
    float delta_output = p_out + i_out + d_out;
    
    pid->output += delta_output;
    
    // ??
    if (pid->output > pid->max_output) pid->output = pid->max_output;
    if (pid->output < -pid->max_output) pid->output = -pid->max_output; // ???????,?????0
    
    pid->prev_error = pid->last_error;
    pid->last_error = pid->error;
    
    return pid->output;
}

// ??? PID/PD ?? (????????)
float PID_Calc_Positional(PID_TypeDef *pid, float target, float feedback) {
    pid->error = target - feedback;
    
    // ????/?? (??)
    pid->integral += pid->error;
    if(pid->integral > pid->max_integral) pid->integral = pid->max_integral;
    if(pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;

    float p_out = pid->kp * pid->error;
    float i_out = pid->ki * pid->integral;
    float d_out = pid->kd * (pid->error - pid->last_error);
    
    pid->output = p_out + i_out + d_out;
    
    // ????
    if (pid->output > pid->max_output) pid->output = pid->max_output;
    if (pid->output < -pid->max_output) pid->output = -pid->max_output;

    pid->last_error = pid->error;

    return pid->output;
}

// ͳһתΪ [-1,1] ƫֵ
float PID_Normalize(float value, float min_value, float max_value)
{
    if(max_value == min_value)
    {
        return 0;
    }

    float clamped = value;
    if(value > max_value) clamped = max_value;
    if(value < min_value) clamped = min_value;

    float span = max_value - min_value;
    float normalized = (clamped - min_value) / span; // 0~1
    return (normalized * 2.0f) - 1.0f;               // -1~1
}

// ֱ·ֹ
float PID_ApplyDeadZone(float value, float dead_zone)
{
    if(value > -dead_zone && value < dead_zone)
    {
        return 0.0f;
    }
    return value;
}

// ͸﻿
float PID_LowPassFilter(float input, float last_output, float alpha)
{
    if(alpha < 0.0f) alpha = 0.0f;
    if(alpha > 1.0f) alpha = 1.0f;
    return alpha * input + (1.0f - alpha) * last_output;
}

// ַжϣ﻿Ҫ﻿ͳ﻿ hold_counter ﻿̳жϣ﻿ֵ﻿ծʱ﻿Ϊ true
bool PID_DetectFinishLine(const uint16_t *samples, uint8_t length, uint16_t threshold, uint16_t hold_need, uint16_t *hold_counter)
{
    if(NULL == samples || NULL == hold_counter)
    {
        return false;
    }

    bool over_threshold = true;
    for(uint8_t i = 0; i < length; i++)
    {
        if(samples[i] < threshold)
        {
            over_threshold = false;
            break;
        }
    }

    if(over_threshold)
    {
        if(*hold_counter < hold_need)
        {
            (*hold_counter)++;
        }
    }
    else
    {
        *hold_counter = 0;
    }

    return (*hold_counter >= hold_need);
}

// ؽڵжϣٱչ﻿Ըϵ﻿ӣ﻿ʹ﻿ hold_counter ȷʱ﻿ hold_need ʱ﻿á﻿cooldown_counter ֤һ﻿
bool PID_DetectRoundabout(const uint16_t *samples,
                          uint8_t length,
                          uint16_t threshold,
                          uint16_t debounce_need,
                          uint16_t hold_need,
                          uint16_t *hold_counter,
                          uint16_t *cooldown_counter)
{
    if(NULL == samples || NULL == hold_counter || NULL == cooldown_counter)
    {
        return false;
    }

    if(*cooldown_counter > 0)
    {
        (*cooldown_counter)--;
        return false;
    }

    uint8_t over_count = 0;
    for(uint8_t i = 0; i < length; i++)
    {
        if(samples[i] > threshold)
        {
            over_count++;
        }
    }

    bool strong_signal = (over_count >= (length / 2));

    if(strong_signal)
    {
        if(*hold_counter < hold_need)
        {
            (*hold_counter)++;
        }
    }
    else
    {
        *hold_counter = 0;
    }

    if(*hold_counter >= hold_need && *hold_counter >= debounce_need)
    {
        *cooldown_counter = hold_need; // ʹſ﻿﻿﻿﻿
        return true;
    }

    return false;
}

// ݵϵ﻿ٶȷֲ
int8_t PID_SegmentSpeed(float normalized_error,
                        int8_t straight_speed,
                        int8_t mild_curve_speed,
                        int8_t sharp_curve_speed,
                        int8_t roundabout_speed,
                        float dead_zone)
{
    float magnitude = normalized_error;
    if(magnitude < 0) magnitude = -magnitude;

    if(magnitude < dead_zone)
    {
        return straight_speed;
    }

    if(magnitude < (dead_zone * 3.0f))
    {
        return mild_curve_speed;
    }

    if(magnitude < 0.9f)
    {
        return sharp_curve_speed;
    }

    return roundabout_speed;
}