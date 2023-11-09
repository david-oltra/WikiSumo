#ifndef __START_MODE__
#define __START_MODE__

#include <stdint.h>
#include <driver/gpio.h>

class START_MODE
{
    public:

        START_MODE(gpio_num_t _PIN_ANALOG);

        void Init();
        int8_t read();

    private:
        gpio_num_t PIN_ANALOG;
};

#endif