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

void set_servo_pwm()
{			
    // 1. ?? PD ??
    // P? = ?? * ??
    // D? = ?? * (???? - ????)
    float p_out = kp * normalized_error;
    float d_out = kd * (normalized_error - last_adc_error);
    
    // 2. ?????? (?? - ???)
    servo_motor_angle = SERVO_MOTOR_M - (p_out + d_out);
    
    // 3. ?????????
    last_adc_error = normalized_error;

    // 4. ?? (????????)
    if(servo_motor_angle > SERVO_MOTOR_R_MAX) servo_motor_angle = SERVO_MOTOR_R_MAX;
    if(servo_motor_angle < SERVO_MOTOR_L_MAX) servo_motor_angle = SERVO_MOTOR_L_MAX;

    // 5. ?? PWM
    pwm_set_duty(SERVO_MOTOR1_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
    pwm_set_duty(SERVO_MOTOR2_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
    pwm_set_duty(SERVO_MOTOR3_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
}