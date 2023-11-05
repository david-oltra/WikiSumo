#include <driver/gpio.h>
#include <motores.h>

MOTORES::MOTORES(gpio_num_t _MOTOR_A_1, gpio_num_t _MOTOR_A_2, gpio_num_t _MOTOR_B_1, gpio_num_t _MOTOR_B_2, gpio_num_t _MOTOR_A_PWM, gpio_num_t _MOTOR_B_PWM)
{
    MOTOR_A_1 = _MOTOR_A_1;
    MOTOR_A_2 = _MOTOR_A_2;
    MOTOR_B_1 = _MOTOR_B_1;
    MOTOR_B_2 = _MOTOR_B_2;
    MOTOR_A_PWM = _MOTOR_A_PWM;
    MOTOR_B_PWM = _MOTOR_B_PWM;
}

void MOTORES::Init()
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

void MOTORES::Update(int8_t dir_motor_A_1, int8_t dir_motor_A_2, int8_t dir_motor_B_1, int8_t dir_motor_B_2)
{
    gpio_set_level(MOTOR_A_1,dir_motor_A_1);
    gpio_set_level(MOTOR_A_1,dir_motor_A_2);
    gpio_set_level(MOTOR_B_1,dir_motor_B_1);
    gpio_set_level(MOTOR_B_2,dir_motor_B_2);
}