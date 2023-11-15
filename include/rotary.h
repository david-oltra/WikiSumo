#ifndef __ROTARY_H__
#define __ROTARY_H__

#include <driver/gpio.h>

class ROTARY
{
    public:

        ROTARY(gpio_num_t _PIN_S1, gpio_num_t _PIN_S2, gpio_num_t _PIN_KEY);

        void Init();

        uint8_t CheckEncoder();
        
        bool Key();
 
    private:
        gpio_num_t PIN_S1;
        gpio_num_t PIN_S2;
        gpio_num_t PIN_KEY;

};

#endif