#include "zf_common_headfile.h"
#include "my_common.h"

// ------------------ ?????? (?????) ------------------

// ????????????
int16 encoder_data_1 = 0;
int16 encoder_data_2 = 0;
int16 encoder_data_3 = 0;
int16 encoder_data_4 = 0;

// ??????ADC????
uint8 channel_index = 0;
adc_channel_enum channel_list[ADC_CHANNEL_NUMBER] = 
{
    ADC_CHANNEL1, ADC_CHANNEL2, ADC_CHANNEL3, ADC_CHANNEL4,
};
uint16_t adc_buffer[ADC_CHANNEL_NUMBER];

// ??????????????
float adc_error;

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
    adc_error = (float)(adc_buffer[0] - adc_buffer[3]);
}

// ????????
void get_data()
{
    get_encoder();
    get_adc();
    
    // roundabout_detect();  // ????????,????
}