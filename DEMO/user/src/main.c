
// ****************************  ****************************

//2025.11.25 ѧϰ

#include "zf_common_headfile.h"
#include "my_common.h"

// µĹ̻߹ƶλִ²
// һ رд򿪵ļ
// ڶ project->clean  ȴ·
extern uint16_t adc_buffer[ADC_CHANNEL_NUMBER];
extern int16 encoder_data_1;
extern float adc_error;
extern int8 duty;         // ???? duty

void Init_All(void);
extern uint8_t roundabout_detected;
extern uint8_t finish_detected;
int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);  // ɾ
    debug_init();                   // Զ˿ڳʼ

    system_delay_ms(300);           //ȴϵ
        Init_All();                                             //ʼ



        //ŻڡƵӦ죬Ƶʼ㸺Сڸ㷨
        pit_ms_init(PIT_CH, CONTROL_PERIOD_MS);// ״̬ʱ

        tft180_show_string(20, 0, "DianCi_demo");
        tft180_show_string(0, 15, "ADC1:");
        tft180_show_string(0, 30, "ADC2:");
        tft180_show_string(0, 45, "ADC3:");
        tft180_show_string(0, 60, "ADC4:");
        tft180_show_string(0, 75, "ERROR");
        tft180_show_string(0, 90, "ENCODER:");
        tft180_show_string(0, 105, "DUTY:");
        tft180_show_string(0, 120, "FLAG:");

    while(1)
    {
        // ˴дҪѭִеĴ
        tft180_show_int(50, 15, adc_buffer[0], 4);
        tft180_show_int(50, 30, adc_buffer[1], 4);
                tft180_show_int(50, 45, adc_buffer[2], 4);
                tft180_show_int(50, 60, adc_buffer[3], 4);

                tft180_show_float(50, 75, adc_error, 4, 1);

                tft180_show_int(70, 90, encoder_data_1, 4);

// ???
tft180_show_string(0, 105, "DUTY:");       // ???? DUTY
tft180_show_int(50, 105, duty, 3);         // ???? duty ??

        tft180_show_int(40, 120, finish_detected, 1);
        tft180_show_int(60, 120, roundabout_detected, 1);
        // ˴дҪѭִеĴ
    }
}

//˳ʼʼ4·ӿڣ4·ӿڣ4·ADCԼ3·ӿ
void Init_All(void)
{
        //ʼDRV8701
        gpio_init(MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO ʼΪ Ĭ
    pwm_init(MOTOR1_PWM, 17000, 0);                                                  // PWM ͨʼƵ 17KHz ռձȳʼΪ 0

    gpio_init(MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO ʼΪ Ĭ
    pwm_init(MOTOR2_PWM, 17000, 0);                                                  // PWM ͨʼƵ 17KHz ռձȳʼΪ 0

    gpio_init(MOTOR3_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO ʼΪ Ĭ
    pwm_init(MOTOR3_PWM, 17000, 0);                                                  // PWM ͨʼƵ 17KHz ռձȳʼΪ 0

    gpio_init(MOTOR4_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO ʼΪ Ĭ
    pwm_init(MOTOR4_PWM, 17000, 0);                                                  // PWM ͨʼƵ 17KHz ռձȳʼΪ 0

        //ݱͺ͸ѡʼ
        //ʼ
    encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B); // ʼģ ģʽ
    encoder_quad_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B); // ʼģ ģʽ
    encoder_quad_init(ENCODER_3, ENCODER_3_A, ENCODER_3_B); // ʼģ ģʽ
    encoder_quad_init(ENCODER_4, ENCODER_4_A, ENCODER_4_B); // ʼģ ģʽ

        //ʼ
//    encoder_dir_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B); // ʼģ
//    encoder_dir_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B); // ʼģ
//    encoder_dir_init(ENCODER_3, ENCODER_3_A, ENCODER_3_B); // ʼģ
//    encoder_dir_init(ENCODER_4, ENCODER_4_A, ENCODER_4_B); // ʼģ

        //ʼadc
        adc_init(ADC_CHANNEL1, ADC_12BIT);                                          // ʼӦ ADC ͨΪӦ
    adc_init(ADC_CHANNEL2, ADC_12BIT);                                          // ʼӦ ADC ͨΪӦ
    adc_init(ADC_CHANNEL3, ADC_12BIT);                                          // ʼӦ ADC ͨΪӦ
    adc_init(ADC_CHANNEL4, ADC_12BIT);                                          // ʼӦ ADC ͨΪӦ

        //ʼ
    pwm_init(SERVO_MOTOR1_PWM, SERVO_MOTOR_FREQ, 0);
    pwm_init(SERVO_MOTOR2_PWM, SERVO_MOTOR_FREQ, 0);
    pwm_init(SERVO_MOTOR3_PWM, SERVO_MOTOR_FREQ, 0);

        //ʼĻ
        tft180_init();
        tft180_set_dir(TFT180_CROSSWISE);

}

extern void get_data();
extern void set_servo_pwm();
extern void set_speed_pwm();

//жϺгʼΪÿ10msִһ
void pit_handler (void)
{

        get_data();//ȡ
        set_servo_pwm();//ö
        set_speed_pwm();//õٶ

}

