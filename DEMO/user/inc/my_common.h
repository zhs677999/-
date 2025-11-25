#ifndef _mycommon_h
#define _mycommon_h

#include "zf_common_headfile.h"
#include "math.h"
#include "pid.h"

// ʹõжϱ ޸ ҪͬӦ޸жϱ isr.c еĵ
#define PIT_CH                      (PIT_CH0 )
#define CONTROL_PERIOD_MS           (8)
#define LED1 (B9)

//ռձ޷
#define MAX_PWM_DUTY            (50)

//
#define MOTOR1_DIR               (C9)
#define MOTOR1_PWM               (PWM2_MODULE1_CHA_C8)

#define MOTOR2_DIR               (C7)
#define MOTOR2_PWM               (PWM2_MODULE0_CHA_C6)

#define MOTOR3_DIR               (D2)
#define MOTOR3_PWM               (PWM2_MODULE3_CHB_D3)

#define MOTOR4_DIR               (C10)
#define MOTOR4_PWM               (PWM2_MODULE2_CHB_C11)

//
#define ENCODER_1                   (QTIMER1_ENCODER1)
#define ENCODER_1_A                 (QTIMER1_ENCODER1_CH1_C0)
#define ENCODER_1_B                 (QTIMER1_ENCODER1_CH2_C1)

#define ENCODER_2                   (QTIMER1_ENCODER2)
#define ENCODER_2_A                 (QTIMER1_ENCODER2_CH1_C2)
#define ENCODER_2_B                 (QTIMER1_ENCODER2_CH2_C24)

#define ENCODER_3                   (QTIMER2_ENCODER1)
#define ENCODER_3_A                 (QTIMER2_ENCODER1_CH1_C3)
#define ENCODER_3_B                 (QTIMER2_ENCODER1_CH2_C4)

#define ENCODER_4                   (QTIMER2_ENCODER2)
#define ENCODER_4_A                 (QTIMER2_ENCODER2_CH1_C5)
#define ENCODER_4_B                 (QTIMER2_ENCODER2_CH2_C25)

//źadcź
#define ADC_CHANNEL_NUMBER          (4)

#define ADC_CHANNEL1            (ADC1_CH12_B23)
#define ADC_CHANNEL2            (ADC1_CH10_B21)
#define ADC_CHANNEL3            (ADC1_CH4_B15)
#define ADC_CHANNEL4            (ADC1_CH3_B14)

//
#define SERVO_MOTOR1_PWM             (PWM4_MODULE2_CHA_C30)
#define SERVO_MOTOR2_PWM             (PWM1_MODULE3_CHA_D0)
#define SERVO_MOTOR3_PWM             (PWM1_MODULE3_CHB_D1)

#define SERVO_MOTOR_FREQ            (50)

#define SERVO_MOTOR_L_MAX           (80)
#define SERVO_MOTOR_R_MAX           (100)
#define SERVO_MOTOR_M               (90)
#define SERVO_KP                    (0.045f)
#define SERVO_KD                    (0.18f)
#define SERVO_DEADZONE              (0.05f)

#define SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))

#if (SERVO_MOTOR_FREQ<50 || SERVO_MOTOR_FREQ>300)
    #error "SERVO_MOTOR_FREQ ERROE!"
#endif

#define ADC_ERROR_RANGE            (2048.0f)
#define ADC_FILTER_ALPHA           (0.25f)

// ??????
// ???????? - ???
#define ROUNDABOUT_THRESHOLD 800
//#define ROUNDABOUT_SIDE_THRESHOLD 500
#define ROUNDABOUT_DEBOUNCE 25
#define ROUNDABOUT_HOLD_TIME 150
#define ROUNDABOUT_COOLDOWN 300

#define FINISH_THRESHOLD           2000
#define FINISH_HOLD_TIME           60

#define SPEED_STRAIGHT             (36)
#define SPEED_MILD_CURVE           (30)
#define SPEED_SHARP_CURVE          (24)
#define SPEED_ROUNDABOUT           (18)
#define SPEED_DEADZONE             (0.10f)

extern uint8_t finish_detected;
extern float normalized_error;
extern float filtered_error;
extern uint8_t roundabout_detected;
extern uint16_t roundabout_timer;
extern uint16_t roundabout_cooldown;

#endif
