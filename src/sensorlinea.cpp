#include <driver/gpio.h>
#include <sensorlinea.h>

SENSOR_LINEA::SENSOR_LINEA(gpio_num_t _LINEA_I, gpio_num_t _LINEA_D)
{
    LINEA_I = _LINEA_I;
    LINEA_D = _LINEA_D;
}

void SENSOR_LINEA::Init()
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << LINEA_I) | (1ULL << LINEA_D);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
}
