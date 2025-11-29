#include "zf_common_headfile.h"
#include "my_common.h"
#include <math.h>

// ????
float servo_motor_angle = SERVO_MOTOR_M; 

// -----------------------------------------------------------
// PD 调节参数
// -----------------------------------------------------------
// 基于归一化误差重新整定，整体增大舵机响应速度
float kp = 9.0f;
float kd = 3.2f;
// -----------------------------------------------------------

extern float normalized_error;      // 归一化后的左右差值
float last_adc_error = 0;           // 上一次误差（用于 D 项）
static float filtered_error = 0.0f; // 滤波后的误差
static float filtered_delta = 0.0f; // 滤波后的误差变化率

extern uint8_t roundabout_detected;
extern uint16_t roundabout_entry_timer;

void set_servo_pwm()
{
    // 1. 对误差进行滤波和直道死区处理，抑制小幅摆动
    filtered_error = (1.0f - SERVO_ERROR_FILTER_ALPHA) * filtered_error + SERVO_ERROR_FILTER_ALPHA * normalized_error;
    float raw_delta = filtered_error - last_adc_error;
    filtered_delta = (1.0f - SERVO_DELTA_FILTER_ALPHA) * filtered_delta + SERVO_DELTA_FILTER_ALPHA * raw_delta;

    float effective_error = filtered_error;
    if(!roundabout_detected && fabsf(effective_error) < SERVO_STRAIGHT_DEADBAND)
    {
        effective_error = 0.0f;
    }

    // 2. 环岛入口直行保持，避免乱打舵
    if(roundabout_detected && roundabout_entry_timer > 0)
    {
        effective_error = 0.0f;
        filtered_delta = 0.0f;
    }

    // 3. PD + 前馈提前量，提高入弯响应速度
    float predictive = CURVE_LEAD_GAIN * filtered_delta;
    float p_out = kp * effective_error;
    float d_out = kd * filtered_delta;

    float target_angle = SERVO_MOTOR_M - (p_out + d_out + predictive);

    // 4. 输出平滑，降低舵机抖动
    servo_motor_angle = (1.0f - SERVO_OUTPUT_SMOOTH_ALPHA) * servo_motor_angle + SERVO_OUTPUT_SMOOTH_ALPHA * target_angle;

    // 5. 存储误差供下一次 D 项使用
    last_adc_error = filtered_error;

    // 6. 限幅（保护舵机）
    if(servo_motor_angle > SERVO_MOTOR_R_MAX) servo_motor_angle = SERVO_MOTOR_R_MAX;
    if(servo_motor_angle < SERVO_MOTOR_L_MAX) servo_motor_angle = SERVO_MOTOR_L_MAX;

    // 7. 输出 PWM
    pwm_set_duty(SERVO_MOTOR1_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
    pwm_set_duty(SERVO_MOTOR2_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
    pwm_set_duty(SERVO_MOTOR3_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
}
