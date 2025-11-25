#include "zf_common_headfile.h"
#include "my_common.h"
#include "pid.h"

// ------------------ ?????? (?????) ------------------

// ????????????
int16 encoder_data_1 = 0;
int16 encoder_data_2 = 0;
int16 encoder_data_3 = 0;
int16 encoder_data_4 = 0;

uint8_t finish_detected = 0;
uint8_t roundabout_detected = 0;
uint16_t roundabout_timer = 0;
uint16_t roundabout_cooldown = 0;

// ??????ADC????
uint8 channel_index = 0;
adc_channel_enum channel_list[ADC_CHANNEL_NUMBER] = 
{
    ADC_CHANNEL1, ADC_CHANNEL2, ADC_CHANNEL3, ADC_CHANNEL4,
};
uint16_t adc_buffer[ADC_CHANNEL_NUMBER];

// ??????????????
float adc_error;
float filtered_error = 0.0f;
float normalized_error = 0.0f;

// -------------------------------------------------------------

// ???????
void get_encoder()
{
    encoder_data_1 = encoder_get_count(ENCODER_1);
    encoder_clear_count(ENCODER_1);

    encoder_data_2 = encoder_get_count(ENCODER_2);
    encoder_clear_count(ENCODER_2);
    
    encoder_data_3 = encoder_get_count(ENCODER_3);
    encoder_clear_count(ENCODER_3);

    encoder_data_4 = encoder_get_count(ENCODER_4);
    encoder_clear_count(ENCODER_4);
}

// ????ADC??
void get_adc()
{
    // ????4???
    for(channel_index = 0; channel_index < ADC_CHANNEL_NUMBER; channel_index ++)
    {
        adc_buffer[channel_index] = adc_convert(channel_list[channel_index]);
    }

    // ????:????(0) - ????(3)
    // ??????,?????,error > 0
    // ??????,?????,error < 0
    float raw_error = (float)(adc_buffer[0] - adc_buffer[3]);

    // ?????????
    filtered_error = PID_LowPassFilter(raw_error, filtered_error, ADC_FILTER_ALPHA);
    adc_error = filtered_error;

    // ???? -1 ~ 1
    normalized_error = PID_Normalize(filtered_error, -ADC_ERROR_RANGE, ADC_ERROR_RANGE);
}

// ????????
void get_data()
{
    get_encoder();
    get_adc();

    static uint16_t finish_hold_counter = 0;
    if(!finish_detected)
    {
        finish_detected = PID_DetectFinishLine(adc_buffer,
                                               ADC_CHANNEL_NUMBER,
                                               FINISH_THRESHOLD,
                                               FINISH_HOLD_TIME,
                                               &finish_hold_counter);
    }

    if(!roundabout_detected)
    {
        roundabout_detected = PID_DetectRoundabout(adc_buffer,
                                                   ADC_CHANNEL_NUMBER,
                                                   ROUNDABOUT_THRESHOLD,
                                                   ROUNDABOUT_DEBOUNCE,
                                                   ROUNDABOUT_HOLD_TIME,
                                                   &roundabout_timer,
                                                   &roundabout_cooldown);
    }
    else if(roundabout_cooldown > 0)
    {
        roundabout_cooldown--;
    }
    else
    {
        roundabout_detected = 0;
    }
}