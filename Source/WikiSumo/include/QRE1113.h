#ifndef __QRE1113_H__
#define __QRE1113_H__

#include <driver/gpio.h>

class QRE1113
{
    public:
        QRE1113(gpio_num_t _PIN_SENSOR);

        void Init();

        gpio_num_t PinNum();

    private:
        gpio_num_t PIN_SENSOR;

};

#endif