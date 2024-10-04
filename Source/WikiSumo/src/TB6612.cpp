#include <driver/gpio.h>
#include <TB6612.h>

TB6612::TB6612(gpio_num_t _MOTOR_A_1, gpio_num_t _MOTOR_A_2, gpio_num_t _MOTOR_A_PWM, gpio_num_t _MOTOR_B_1, gpio_num_t _MOTOR_B_2, gpio_num_t _MOTOR_B_PWM)
{
    MOTOR_A_1 = _MOTOR_A_1;
    MOTOR_A_2 = _MOTOR_A_2;
    MOTOR_B_1 = _MOTOR_B_1;
    MOTOR_B_2 = _MOTOR_B_2;
    MOTOR_A_PWM = _MOTOR_A_PWM;
    MOTOR_B_PWM = _MOTOR_B_PWM;
}

void TB6612::Init()
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << MOTOR_A_1) | (1ULL << MOTOR_A_2) | (1ULL << MOTOR_A_PWM) | (1ULL << MOTOR_B_1) | (1ULL << MOTOR_B_2) | (1ULL << MOTOR_B_PWM);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    gpio_set_level(MOTOR_A_1, 0);
    gpio_set_level(MOTOR_A_2, 0);
    gpio_set_level(MOTOR_A_PWM, 1);
    gpio_set_level(MOTOR_B_1, 0);
    gpio_set_level(MOTOR_B_2, 0);
    gpio_set_level(MOTOR_B_PWM, 1);
}

void TB6612::Update(enum DIRECTION dir)
{
    switch(dir)
    {
        case TB6612::STOP:
            gpio_set_level(MOTOR_A_1,1);
            gpio_set_level(MOTOR_A_2,1);
            gpio_set_level(MOTOR_B_1,1);
            gpio_set_level(MOTOR_B_2,1);
            break;
        case TB6612::FWD:
            gpio_set_level(MOTOR_A_1,0);
            gpio_set_level(MOTOR_A_2,1);
            gpio_set_level(MOTOR_B_1,1);
            gpio_set_level(MOTOR_B_2,0);
            break;
        case TB6612::LEFT:
            gpio_set_level(MOTOR_A_1,0);
            gpio_set_level(MOTOR_A_2,1);
            gpio_set_level(MOTOR_B_1,0);
            gpio_set_level(MOTOR_B_2,1);
            break;
        case TB6612::RIGHT:
            gpio_set_level(MOTOR_A_1,1);
            gpio_set_level(MOTOR_A_2,0);
            gpio_set_level(MOTOR_B_1,1);
            gpio_set_level(MOTOR_B_2,0);
            break;
        default:
            break;
    }
}