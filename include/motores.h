#ifndef __MOTORES_H__
#define __MOTORES_H__

#include <driver/gpio.h>

class MOTORES
{
    private:
        gpio_num_t MOTOR_A_1, MOTOR_A_2, MOTOR_B_1, MOTOR_B_2, MOTOR_A_PWM, MOTOR_B_PWM;

    public:
        MOTORES(gpio_num_t, gpio_num_t, gpio_num_t, gpio_num_t, gpio_num_t, gpio_num_t);

        void Init();
        void Update(int8_t, int8_t, int8_t, int8_t);
        /*
        1 1 1 1 -> STOP
        0 1 0 1 -> FWD
        1 0 1 0 -> BACK
        1 0 0 1 -> IZQ
        0 1 1 0 -> DRCH
        */

};

#endif