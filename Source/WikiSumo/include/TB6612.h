#ifndef __TB6612_H__
#define __TB6612_H__


class TB6612
{
    public:

        enum DIRECTION
        {
            STOP = 0,
            FWD,
            LEFT,
            RIGHT,
            BACK
        };

        TB6612(gpio_num_t _MOTOR_A_1, gpio_num_t _MOTOR_A_2, gpio_num_t _MOTOR_A_PWM, 
               gpio_num_t _MOTOR_B_1, gpio_num_t _MOTOR_B_2, gpio_num_t _MOTOR_B_PWM);

        void Init();

        void Update(enum DIRECTION dir);
        /*
        1 1 1 1 -> STOP
        0 1 0 1 -> FWD
        1 0 1 0 -> BACK
        1 0 0 1 -> IZQ
        0 1 1 0 -> DRCH
        */

    private:
        gpio_num_t MOTOR_A_1, MOTOR_A_2, MOTOR_B_1, MOTOR_B_2, MOTOR_A_PWM, MOTOR_B_PWM;

};

#endif