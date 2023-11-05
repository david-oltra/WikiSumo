#include <startmode.h>
#include "driver/adc.h"

int8_t start_mode()
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    
    // Get configuration
    int8_t num_muestras = 10;
    uint32_t adc_raw=0;
    for(uint8_t i =0; i < num_muestras; i++){
        adc_raw += adc1_get_raw(ADC1_CHANNEL_6);
    }
    adc_raw /= num_muestras;

    int8_t mode = -1;

    if(adc_raw < 200) // IZQ
    {
        mode = 0;
    }
    else if(adc_raw < 1506) // FRONT
    {
        mode = 1;
    }
    else if ( adc_raw < 2065) // DRCH
    {
        mode = 2;
    }
    else if ( adc_raw < 2441) // TRAS
    {
        mode = 3;
    }
    else //1XXX
    {
        // Intentionally empty
    }

    return mode;
}