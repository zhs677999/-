#ifndef _PID_H_
#define _PID_H_

#include "zf_common_headfile.h"
#include <stdbool.h>

typedef struct {
    float kp;           // ????
    float ki;           // ????
    float kd;           // ????
    
    float error;        // ????
    float last_error;   // ?????
    float prev_error;   // ?????(?????)
    
    float integral;     // ?????(?????)
    float output;       // PID???
    
    float max_output;   // ????
    float max_integral; // ????
} PID_TypeDef;

// ????
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float max_out, float max_int);
float PID_Calc_Positional(PID_TypeDef *pid, float target, float feedback); // ???(????,??????PD)
float PID_Calc_Incremental(PID_TypeDef *pid, float target, float feedback); // ???(??????)

// ͳһȡֵ [-1,1]
float PID_Normalize(float value, float min_value, float max_value);

// ͨͷõļ﻿
float PID_ApplyDeadZone(float value, float dead_zone);

// ̬͸
float PID_LowPassFilter(float input, float last_output, float alpha);

// ݽ﻿﻿﻿ж﻿﻿﻿﻿﻿ֵʱ﻿Ի﻿﻿﻿﻿չޡ
bool PID_DetectFinishLine(const uint16_t *samples, uint8_t length, uint16_t threshold, uint16_t hold_need, uint16_t *hold_counter);

// ؽڵж﻿﻿﻿Ѵ﻿﻿﻿ֵʱڴ﻿ر﻿սλ﻿ʱ롢﻿ջ﻿ʱ롣
bool PID_DetectRoundabout(const uint16_t *samples,
                          uint8_t length,
                          uint16_t threshold,
                          uint16_t debounce_need,
                          uint16_t hold_need,
                          uint16_t *hold_counter,
                          uint16_t *cooldown_counter);

// ݷٶȷ﻿ֲ﻿޶ȡ﻿﻿﻿ֵ﻿ʾֱ·ͬʱ﻿Ƶȵķ﻿﻿﻿ޡ
int8_t PID_SegmentSpeed(float normalized_error,
                        int8_t straight_speed,
                        int8_t mild_curve_speed,
                        int8_t sharp_curve_speed,
                        int8_t roundabout_speed,
                        float dead_zone);

#endif