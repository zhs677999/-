#include "zf_common_headfile.h"
#include "my_common.h"

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
extern uint8_t roundabout_entry_active;

static float servo_target_angle = SERVO_MOTOR_M;

void set_servo_pwm()
{
    float abs_error = fabsf(normalized_error);
    float effective_error = normalized_error;

    if(roundabout_entry_active)
    {
        servo_target_angle = SERVO_MOTOR_M;
        last_adc_error = 0.0f;
    }
    else
    {
        if(abs_error < SERVO_STRAIGHT_ERROR_DEADBAND)
        {
            effective_error = 0.0f;
        }

        float kp_dynamic = kp;
        float kd_dynamic = kd;

        if(abs_error > SERVO_CURVE_GAIN_THRESHOLD)
        {
            kp_dynamic *= SERVO_CURVE_KP_BOOST;
            kd_dynamic *= SERVO_CURVE_KD_BOOST;
        }

        float p_out = kp_dynamic * effective_error;
        float d_out = kd_dynamic * (effective_error - last_adc_error);

        servo_target_angle = SERVO_MOTOR_M - (p_out + d_out);
        last_adc_error = effective_error;
    }

    if(servo_target_angle > SERVO_MOTOR_R_MAX) servo_target_angle = SERVO_MOTOR_R_MAX;
    if(servo_target_angle < SERVO_MOTOR_L_MAX) servo_target_angle = SERVO_MOTOR_L_MAX;

    float smooth_alpha = (abs_error < SERVO_CURVE_GAIN_THRESHOLD) ? SERVO_SMOOTH_ALPHA_STRAIGHT : SERVO_SMOOTH_ALPHA_CURVE;
    servo_motor_angle = smooth_alpha * servo_motor_angle + (1.0f - smooth_alpha) * servo_target_angle;

    if(servo_motor_angle > SERVO_MOTOR_R_MAX) servo_motor_angle = SERVO_MOTOR_R_MAX;
    if(servo_motor_angle < SERVO_MOTOR_L_MAX) servo_motor_angle = SERVO_MOTOR_L_MAX;

    pwm_set_duty(SERVO_MOTOR1_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
    pwm_set_duty(SERVO_MOTOR2_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
    pwm_set_duty(SERVO_MOTOR3_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
}