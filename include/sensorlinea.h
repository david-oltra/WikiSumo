#ifndef __SENSORLINEA_H__
#define __SENSORLINEA_H__

#include <driver/gpio.h>

class SENSOR_LINEA
{
    private:
        gpio_num_t LINEA_I, LINEA_D;

    public:
        SENSOR_LINEA(gpio_num_t, gpio_num_t);

        void Init();

};

#endif