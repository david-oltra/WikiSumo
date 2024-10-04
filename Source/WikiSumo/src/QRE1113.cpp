#include <driver/gpio.h>
#include <QRE1113.h>

QRE1113::QRE1113(gpio_num_t _PIN_SENSOR)
{
    PIN_SENSOR = _PIN_SENSOR;
}

void QRE1113::Init()
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << PIN_SENSOR);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
}


gpio_num_t QRE1113::PinNum()
{
    return PIN_SENSOR;
}