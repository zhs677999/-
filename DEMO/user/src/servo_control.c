#include "zf_common_headfile.h"
#include "my_common.h"

// ????
float servo_motor_angle = SERVO_MOTOR_M; 

extern float normalized_error;
static float last_control_error = 0;

void set_servo_pwm()
{			
    // 1. ?? PD ??
    // P? = ?? * ??
    // D? = ?? * (???? - ????)
    float control_error = PID_ApplyDeadZone(normalized_error, SERVO_DEADZONE);

    float p_out = SERVO_KP * control_error;
    float d_out = SERVO_KD * (control_error - last_control_error);

    // 2. ?????? (?? - ???)
    servo_motor_angle = SERVO_MOTOR_M - (p_out + d_out);

    // 3. ?????????
    last_control_error = control_error;

    // 4. ?? (????????)
    if(servo_motor_angle > SERVO_MOTOR_R_MAX) servo_motor_angle = SERVO_MOTOR_R_MAX;
    if(servo_motor_angle < SERVO_MOTOR_L_MAX) servo_motor_angle = SERVO_MOTOR_L_MAX;

    // 5. ?? PWM
    pwm_set_duty(SERVO_MOTOR1_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
    pwm_set_duty(SERVO_MOTOR2_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
    pwm_set_duty(SERVO_MOTOR3_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
}