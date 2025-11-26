//2025.11.25 ѧϰ

#include "zf_common_headfile.h"
#include "my_common.h"

extern uint16_t adc_buffer[ADC_CHANNEL_NUMBER];
extern int16 encoder_data_1;
extern float normalized_error;
extern uint8_t finish_detected;
extern uint8_t roundabout_detected;
extern int8 duty;         // 当前 duty

void Init_All(void);

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();

    system_delay_ms(300);
    Init_All();

    // 控制周期调整为可配置，默认 5ms
    pit_ms_init(PIT_CH, CONTROL_PERIOD_MS);

    // 初始界面
    tft180_show_string(20, 0, "DianCi_demo");
    tft180_show_string(0, 15, "ADC1:");
    tft180_show_string(0, 30, "ADC2:");
    tft180_show_string(0, 45, "ADC3:");
    tft180_show_string(0, 60, "ADC4:");
    tft180_show_string(0, 75, "ERR(N):");
    tft180_show_string(0, 90, "ENCODER:");
    tft180_show_string(0, 105, "DUTY:");
    tft180_show_string(0, 120, "ROUND:");
    tft180_show_string(0, 135, "FINISH:");

    while(1)
    {
        // 循环显示关键调试数据
        tft180_show_int(50, 15, adc_buffer[0], 4);
        tft180_show_int(50, 30, adc_buffer[1], 4);
        tft180_show_int(50, 45, adc_buffer[2], 4);
        tft180_show_int(50, 60, adc_buffer[3], 4);

        tft180_show_float(50, 75, normalized_error, 4, 2);

        tft180_show_int(70, 90, encoder_data_1, 4);

        tft180_show_int(50, 105, duty, 3);

        tft180_show_int(60, 120, roundabout_detected, 1);
        tft180_show_int(60, 135, finish_detected, 1);
    }
}

// 初始化硬件资源
void Init_All(void)
{
    // DRV8701
    gpio_init(MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR1_PWM, 17000, 0);

    gpio_init(MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR2_PWM, 17000, 0);

    gpio_init(MOTOR3_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR3_PWM, 17000, 0);

    gpio_init(MOTOR4_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR4_PWM, 17000, 0);

    // 编码器
    encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B);
    encoder_quad_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B);
    encoder_quad_init(ENCODER_3, ENCODER_3_A, ENCODER_3_B);
    encoder_quad_init(ENCODER_4, ENCODER_4_A, ENCODER_4_B);

    // ADC
    adc_init(ADC_CHANNEL1, ADC_12BIT);
    adc_init(ADC_CHANNEL2, ADC_12BIT);
    adc_init(ADC_CHANNEL3, ADC_12BIT);
    adc_init(ADC_CHANNEL4, ADC_12BIT);

    // 舵机 PWM
    pwm_init(SERVO_MOTOR1_PWM, SERVO_MOTOR_FREQ, 0);
    pwm_init(SERVO_MOTOR2_PWM, SERVO_MOTOR_FREQ, 0);
    pwm_init(SERVO_MOTOR3_PWM, SERVO_MOTOR_FREQ, 0);

    // LCD
    tft180_init();
    tft180_set_dir(TFT180_CROSSWISE);
}

extern void get_data();
extern void set_servo_pwm();
extern void set_speed_pwm();

// PIT 中断服务函数，每个控制周期调用
void pit_handler (void)
{
    get_data();
    set_servo_pwm();
    set_speed_pwm();
}
