#include <driver/gpio.h>
#include <esp_timer.h>
#include <led.h>

LED::LED(gpio_num_t _PIN_LED)
{
    PIN_LED = _PIN_LED;
}

void LED::Init()
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << PIN_LED);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(PIN_LED,1);
}

uint8_t led_status = 0;
int64_t timer_ref_led = esp_timer_get_time();
int64_t timer_led;
int32_t freq;

void LED::Update(enum STATUS st)
{
    switch(st)
    {
        case LED::OFF:
            gpio_set_level(PIN_LED,0);
            break;
        case LED::ON:
            gpio_set_level(PIN_LED,1);
            break;
        case LED::BLINK_250ms:
            freq = 250;
            break;
        case LED::BLINK_500ms:
            freq = 500;
            break;
        case LED::BLINK_1s:
            freq = 1000;
            break;
        default:
            break;
    }

    timer_led = esp_timer_get_time() - timer_ref_led;
    if (timer_led > freq)
    {
        timer_led = esp_timer_get_time() - timer_ref_led;
        led_status = !led_status;
        gpio_set_level(PIN_LED,led_status);
        timer_ref_led = esp_timer_get_time();
    }
}

