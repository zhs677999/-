#include "zf_common_headfile.h"
#include "my_common.h"

// ?????(??),?? 0-100
int8 duty = SPEED_STRAIGHT;

extern float normalized_error;
extern uint8_t roundabout_detected;

void set_speed_pwm()
{
    int32_t final_pwm;
    
    // ???????:??? 0-100 ? duty ??? PWM ??
    int8_t target_speed = PID_SegmentSpeed(normalized_error,
                                           SPEED_STRAIGHT,
                                           SPEED_MILD_CURVE,
                                           SPEED_SHARP_CURVE,
                                           SPEED_ROUNDABOUT,
                                           SPEED_DEADZONE);

    if(roundabout_detected)
    {
        target_speed = SPEED_ROUNDABOUT;
    }

    duty = target_speed;

    final_pwm = duty * (PWM_DUTY_MAX / 100);
    
    // ???????,????
    if(final_pwm > PWM_DUTY_MAX) final_pwm = PWM_DUTY_MAX;
    if(final_pwm < 0) final_pwm = 0;

    // --- ?? ---
    gpio_set_level(MOTOR1_DIR, GPIO_HIGH);
    pwm_set_duty(MOTOR1_PWM, final_pwm);

    gpio_set_level(MOTOR2_DIR, GPIO_HIGH);
    pwm_set_duty(MOTOR2_PWM, final_pwm);

    // --- ?? (??????????) ---
    gpio_set_level(MOTOR3_DIR, GPIO_HIGH);
    pwm_set_duty(MOTOR3_PWM, final_pwm);

    gpio_set_level(MOTOR4_DIR, GPIO_HIGH);
    pwm_set_duty(MOTOR4_PWM, final_pwm);
}