
extern uint16_t roundabout_cooldown;
        tft180_show_string(0, 120, "ROUND:");
        tft180_show_string(0, 135, "COOL:");
        tft180_show_int(60, 120, roundabout_detected, 1);
        tft180_show_int(50, 135, roundabout_cooldown, 4);
// **************************** 代码区域 ****************************

//2025.11.25 学习版

#include "zf_common_headfile.h"
#include "my_common.h"

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完
extern uint16_t adc_buffer[ADC_CHANNEL_NUMBER];
extern int16 encoder_data_1;
extern float adc_error;
extern int8 duty;         // ???? duty

void Init_All(void);
extern uint8_t roundabout_detected;
int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);  // 不可删除
    debug_init();                   // 调试端口初始化

    system_delay_ms(300);           //等待主板其他外设上电完成
	Init_All();						//初始化所有外设
	
	
	
	//这里可以优化控制周期。高频率相应快，低频率计算负担小，适用于复杂算法
	pit_ms_init(PIT_CH, 10);//开启10ms中断
	
	tft180_show_string(20, 0, "DianCi_demo");
	tft180_show_string(0, 15, "ADC1:");
	tft180_show_string(0, 30, "ADC2:");
	tft180_show_string(0, 45, "ADC3:");
	tft180_show_string(0, 60, "ADC4:");
	tft180_show_string(0, 75, "ERROR");
	tft180_show_string(0, 90, "ENCODER:");
	tft180_show_string(0, 105, "DUTY:");
	
    while(1)
    {
        // 此处编写需要循环执行的代码
        tft180_show_int(50, 15, adc_buffer[0], 4);
        tft180_show_int(50, 30, adc_buffer[1], 4);
		tft180_show_int(50, 45, adc_buffer[2], 4);
		tft180_show_int(50, 60, adc_buffer[3], 4);
		
		tft180_show_float(50, 75, adc_error, 4, 1);
		
		tft180_show_int(70, 90, encoder_data_1, 4);
		
// ???
tft180_show_string(0, 105, "DUTY:");       // ???? DUTY
tft180_show_int(50, 105, duty, 3);         // ???? duty ??
        // 此处编写需要循环执行的代码
    }
}

//此初始化函数共初始化4路电机接口，4路编码器接口，4路电感ADC，以及3路舵机接口
void Init_All(void)
{
	//初始化电机驱动DRV8701
	gpio_init(MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR1_PWM, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0
    
    gpio_init(MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR2_PWM, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0

    gpio_init(MOTOR3_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR3_PWM, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0

    gpio_init(MOTOR4_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR4_PWM, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0

	//根据编码器类型和个数选择初始化函数
	//初始化编码器（正交）
    encoder_quad_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B); // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B); // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER_3, ENCODER_3_A, ENCODER_3_B); // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER_4, ENCODER_4_A, ENCODER_4_B); // 初始化编码器模块与引脚 正交解码编码器模式
	
	//初始化编码器（步方向）
//    encoder_dir_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B); // 初始化编码器模块与引脚
//    encoder_dir_init(ENCODER_2, ENCODER_2_A, ENCODER_2_B); // 初始化编码器模块与引脚
//    encoder_dir_init(ENCODER_3, ENCODER_3_A, ENCODER_3_B); // 初始化编码器模块与引脚
//    encoder_dir_init(ENCODER_4, ENCODER_4_A, ENCODER_4_B); // 初始化编码器模块与引脚
	
	//初始化adc引脚
	adc_init(ADC_CHANNEL1, ADC_12BIT);                                          // 初始化对应 ADC 通道为对应精度
    adc_init(ADC_CHANNEL2, ADC_12BIT);                                          // 初始化对应 ADC 通道为对应精度
    adc_init(ADC_CHANNEL3, ADC_12BIT);                                          // 初始化对应 ADC 通道为对应精度
    adc_init(ADC_CHANNEL4, ADC_12BIT);                                          // 初始化对应 ADC 通道为对应精度
		
	//初始化舵机引脚
    pwm_init(SERVO_MOTOR1_PWM, SERVO_MOTOR_FREQ, 0);
    pwm_init(SERVO_MOTOR2_PWM, SERVO_MOTOR_FREQ, 0);
    pwm_init(SERVO_MOTOR3_PWM, SERVO_MOTOR_FREQ, 0);
		
	//初始化屏幕
	tft180_init();
	tft180_set_dir(TFT180_CROSSWISE);
	
}

extern void get_data();
extern void set_servo_pwm();
extern void set_speed_pwm();

//中断函数，在主函数中初始化为每10ms执行一次
void pit_handler (void)
{
	
	get_data();//获取传感器数据
	set_servo_pwm();//设置舵机打角
	set_speed_pwm();//设置电机速度
	
}

