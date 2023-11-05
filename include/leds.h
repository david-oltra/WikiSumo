#ifndef __LEDS_H__
#define __LEDS_H__

#include <driver/gpio.h>

class LEDS
{
    private:
        gpio_num_t led_i, led_f, led_d, led_t;
        uint8_t led_status;

    public:
        LEDS(gpio_num_t, gpio_num_t, gpio_num_t, gpio_num_t);

        void Init();
        void Update(int8_t, int8_t, int8_t, int8_t, int64_t); //LED_I, LED_F, LED_D, LED_T, FREQ.
 

};

#endif