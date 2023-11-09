#include <startmode.h>
#include "driver/adc.h"

START_MODE::START_MODE(gpio_num_t _PIN_ANALOG)
{
    PIN_ANALOG = _PIN_ANALOG;
}

void START_MODE::Init()
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
}

int8_t START_MODE::read()
{
    int8_t num_muestras = 10;
    uint32_t muestras=0;
    
    for(uint8_t i =0; i < num_muestras; i++){
        muestras += adc1_get_raw(ADC1_CHANNEL_6);
    }
    muestras /= num_muestras;

    int8_t mode = -1;

    if(muestras < 200) // LEFT
    {
        mode = 0;
    }
    else if(muestras < 1506) // FRONT
    {
        mode = 1;
    }
    else if ( muestras < 2065) // RIGHT
    {
        mode = 2;
    }
    else if ( muestras < 2441) // BACK
    {
        mode = 3;
    }
    else //1XXX
    {
        // Intentionally empty
    }

    return mode;
}