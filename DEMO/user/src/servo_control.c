#include "zf_common_headfile.h"
#include "my_common.h"

// ????
float servo_motor_angle = SERVO_MOTOR_M; 

// -----------------------------------------------------------
// PD ??????
// -----------------------------------------------------------
// Kp: ???????????,???????? 0.02 ~ 0.06
float kp = 0.035; 

// Kd: ?????Kp?????,?? Kd ?????????? Kp ? 3~10??
float kd = 0.15;  
// -----------------------------------------------------------

extern float adc_error;      // ????
float last_adc_error = 0;    // ??????? (???? D)

void set_servo_pwm()
{			
    // 1. ?? PD ??
    // P? = ?? * ??
    // D? = ?? * (???? - ????)
    float p_out = kp * adc_error;
    float d_out = kd * (adc_error - last_adc_error);
    
    // 2. ?????? (?? - ???)
    servo_motor_angle = SERVO_MOTOR_M - (p_out + d_out);
    
    // 3. ?????????
    last_adc_error = adc_error;

    // 4. ?? (????????)
    if(servo_motor_angle > SERVO_MOTOR_R_MAX) servo_motor_angle = SERVO_MOTOR_R_MAX;
    if(servo_motor_angle < SERVO_MOTOR_L_MAX) servo_motor_angle = SERVO_MOTOR_L_MAX;

    // 5. ?? PWM
    pwm_set_duty(SERVO_MOTOR1_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
    pwm_set_duty(SERVO_MOTOR2_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
    pwm_set_duty(SERVO_MOTOR3_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
}