#include "zf_common_headfile.h"
#include "my_common.h"

// -----------------------------------------------------------
// 环岛强制转向控制参数
// -----------------------------------------------------------
uint16_t roundabout_force_timer = 0;     // 环岛强制转向计时器
uint8_t roundabout_force_active = 0;     // 强制转向激活标志
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
    static uint8_t roundabout_trigger_latched = 0; // 一次性锁存环岛触发

    // 检测到环岛开始计时，使用锁存避免依赖检测标志持续为真
    if(!roundabout_force_active)
    {
        // 首次检测到环岛时锁存触发
        if(roundabout_detected && !roundabout_trigger_latched)
        {
            roundabout_trigger_latched = 1;
            roundabout_detect_timer = 0;
        }

        if(roundabout_trigger_latched)
        {
            if(roundabout_detect_timer < (ROUNDABOUT_DELAY_MS / CONTROL_PERIOD_MS))
            {
                roundabout_detect_timer++;
            }
            else
            {
                // 延时完成后激活强制左打满
                roundabout_force_active = 1;
                roundabout_force_timer = FORCE_LEFT_DURATION_MS / CONTROL_PERIOD_MS;
                roundabout_detect_timer = 0;
                roundabout_trigger_latched = 0;
            }
        }
        else
        {
            roundabout_detect_timer = 0;
        }
    }
    
    // 强制左打满期间
    if(roundabout_force_active)
    {
        servo_motor_angle = FORCE_LEFT_ANGLE;
        
        if(roundabout_force_timer > 0)
        {
            roundabout_force_timer--;
        }
        else
        {
            // 1秒后结束强制转向
            roundabout_force_active = 0;
            roundabout_completed = 1;      // 标记环岛完成，进入出口检测阶段
            roundabout_exit_timer = 0;
            roundabout_exit_watch_timer = 0;
        }
        
        // 直接设置PWM并返回，跳过PD计算
        pwm_set_duty(SERVO_MOTOR1_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
        pwm_set_duty(SERVO_MOTOR2_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
        pwm_set_duty(SERVO_MOTOR3_PWM, (uint32)SERVO_MOTOR_DUTY(servo_motor_angle));
        return;
    }
		
    // 新增：环岛出口检测逻辑
    if(roundabout_completed && !roundabout_exit_detected)
    {
        // 检测环岛出口条件：传感器信号恢复正常（误差较小）
        if(fabsf(normalized_error) < 0.3f)  // 误差较小表示已离开环岛区域
        {
            if(roundabout_exit_timer < (EXIT_CONFIRM_DURATION_MS / CONTROL_PERIOD_MS))
            {
                roundabout_exit_timer++;
            }
            else
            {
                // 确认已离开环岛，允许重新检测
                roundabout_exit_detected = 1;
                roundabout_completed = 0;  // 重置环岛完成标志
                roundabout_exit_watch_timer = 0;
                roundabout_detect_timer = 0;
                roundabout_trigger_latched = 0;
            }
        }
        else
        {
            // 误差变大，可能还在环岛内，重置计时器
            roundabout_exit_timer = 0;
        }

        // 即使出口迟迟未确认，超时后也自动允许重新检测，避免卡在环岛状态
        if(roundabout_exit_watch_timer < (EXIT_REARM_DURATION_MS / CONTROL_PERIOD_MS))
        {
            roundabout_exit_watch_timer++;
        }
        else
        {
            roundabout_exit_detected = 1;
            roundabout_completed = 0;
            roundabout_exit_timer = 0;
            roundabout_exit_watch_timer = 0;
            roundabout_detect_timer = 0;
            roundabout_trigger_latched = 0;
        }
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