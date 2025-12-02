#include "zf_common_headfile.h"
#include "my_common.h"

// -----------------------------------------------------------
// 环岛强制转向控制参数
// -----------------------------------------------------------
uint16_t roundabout_force_timer = 0;     // 环岛强制转向计时器
uint8_t roundabout_force_active = 0;     // 强制转向激活标志
uint8_t roundabout_state = ROUNDABOUT_STATE_IDLE; // 当前环岛处理阶段
#define ROUNDABOUT_DELAY_MS 1200          // 检测到环岛后延迟时间（毫秒）
#define FORCE_LEFT_DURATION_MS 1500       // 强制左打满持续时间（毫秒）
#define FORCE_LEFT_ANGLE SERVO_MOTOR_L_MAX // 强制左打满角度
#define EXIT_REARM_DURATION_MS 2500       // 最长出口等待时间，超时后自动重启检测

extern uint8_t roundabout_detected;      // 环岛检测标志

// 新增：环岛出口检测参数
uint8_t roundabout_exit_detected = 0;    // 环岛出口检测标志
uint8_t roundabout_completed = 0;
uint16_t roundabout_exit_timer = 0;      // 出口确认计时器
#define EXIT_CONFIRM_DURATION_MS 800     // 出口确认时间（毫秒）
static uint16_t roundabout_exit_watch_timer = 0; // 出口监控总时长计时器
uint8_t roundabout_handling = 0;         // 防止处理环岛期间重复触发



// ????
float servo_motor_angle = SERVO_MOTOR_M; 

// -----------------------------------------------------------
// PD 调节参数
// -----------------------------------------------------------
// 基于归一化误差重新整定，整体增大舵机响应速度
float kp = 75.0f;
float kd = 21.0f;
// -----------------------------------------------------------

extern float normalized_error;      // 归一化后的左右差值
float last_adc_error = 0;           // 上一次误差（用于 D 项）

// 在servocontrol.c中添加非线性处理
void set_servo_pwm()
{

    // ========== 新增：环岛强制转向逻辑 ==========
    static uint16_t roundabout_detect_timer = 0;

    // 环岛处理状态机：确保检测、延时、打角、出口复位各阶段可控
    switch(roundabout_state)
    {
        case ROUNDABOUT_STATE_IDLE:
        {
            if(roundabout_detected)
            {
                roundabout_state = ROUNDABOUT_STATE_DELAY;
                roundabout_detect_timer = 0;
                roundabout_handling = 1;
            }
            break;
        }

        case ROUNDABOUT_STATE_DELAY:
        {
            // 延时计时不依赖检测标志持续为真
            if(roundabout_detect_timer < (ROUNDABOUT_DELAY_MS / CONTROL_PERIOD_MS))
            {
                roundabout_detect_timer++;
            }
            else
            {
                roundabout_state = ROUNDABOUT_STATE_FORCE;
                roundabout_force_active = 1;
                roundabout_force_timer = FORCE_LEFT_DURATION_MS / CONTROL_PERIOD_MS;
                roundabout_detected = 0;   // 触发后清除检测标志，避免速度/蜂鸣器逻辑干扰
                roundabout_timer = 0;
            }
            break;
        }

        case ROUNDABOUT_STATE_FORCE:
        {
            servo_motor_angle = FORCE_LEFT_ANGLE;

            if(roundabout_force_timer > 0)
            {
                roundabout_force_timer--;
            }
            else
            {
                // 打角结束，进入出口确认阶段
                roundabout_force_active = 0;
                roundabout_completed = 1;
                roundabout_exit_timer = 0;
                roundabout_exit_watch_timer = 0;
                roundabout_state = ROUNDABOUT_STATE_EXIT_CHECK;
            }

            pwm_set_duty(SERVO_MOTOR1_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
            pwm_set_duty(SERVO_MOTOR2_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
            pwm_set_duty(SERVO_MOTOR3_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
            return;
        }

        case ROUNDABOUT_STATE_EXIT_CHECK:
        {
            // 检测环岛出口：误差回落或超时即复位
            if(fabsf(normalized_error) < 0.3f)
            {
                if(roundabout_exit_timer < (EXIT_CONFIRM_DURATION_MS / CONTROL_PERIOD_MS))
                {
                    roundabout_exit_timer++;
                }
                else
                {
                    roundabout_exit_detected = 1;
                }
            }
            else
            {
                roundabout_exit_timer = 0;
            }

            if(roundabout_exit_watch_timer < (EXIT_REARM_DURATION_MS / CONTROL_PERIOD_MS))
            {
                roundabout_exit_watch_timer++;
            }
            else
            {
                roundabout_exit_detected = 1; // 超时也允许重新检测
            }

            if(roundabout_exit_detected)
            {
                roundabout_completed = 0;
                roundabout_handling = 0;
                roundabout_state = ROUNDABOUT_STATE_IDLE;
                roundabout_exit_timer = 0;
                roundabout_exit_watch_timer = 0;
            }
            break;
        }
        default:
            roundabout_state = ROUNDABOUT_STATE_IDLE;
            break;
    }


    // ========== 环岛强制转向逻辑结束 ==========
		
		
    // 非线性误差放大，对小误差敏感，对大误差饱和
    float error_gain = 1.0f;
    if (fabsf(normalized_error) > 0.2f) {
        error_gain = 1.5f;  // 大误差时增益更大
    }
    
    float enhanced_error = normalized_error * error_gain;
    
    float p_out = kp * enhanced_error;
    float d_out = kd * (enhanced_error - last_adc_error);
    
    servo_motor_angle = SERVO_MOTOR_M - (p_out + d_out);
    
    // 其余代码保持不变...
    
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