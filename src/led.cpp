#include <driver/gpio.h>
#include <esp_timer.h>
#include <led.h>

int64_t timer_ref_led[GPIO_NUM_MAX];
uint8_t led_status[GPIO_NUM_MAX];

LED::LED(gpio_num_t _PIN_LED)
{
    PIN_LED = _PIN_LED;
}

void LED::Init()
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << PIN_LED);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    timer_ref_led[PIN_LED] = esp_timer_get_time();
    led_status[PIN_LED] = 1;

    gpio_set_level(PIN_LED,led_status[PIN_LED]);
}

int64_t timer_led;
int64_t freq;

void LED::Update(enum STATUS st)
{
    switch(st)
    {
        case OFF:
            gpio_set_level(PIN_LED,0);
            freq=0;
            break;
        case LED::ON:
            gpio_set_level(PIN_LED,1);
            freq=0;
            break;
        case LED::BLINK_250ms:
            freq = 250000;
            break;
        case LED::BLINK_500ms:
            freq = 500000;
            break;
        case LED::BLINK_1s:
            freq = 1000000;
            break;
        default:
            break;
    }

    if(freq!=0)
    {
        timer_led = esp_timer_get_time() - timer_ref_led[PIN_LED];
        if (timer_led > freq)
        {
            timer_led = esp_timer_get_time() - timer_ref_led[PIN_LED];
            led_status[PIN_LED] = !led_status[PIN_LED];
            gpio_set_level(PIN_LED,led_status[PIN_LED]);
            timer_ref_led[PIN_LED] = esp_timer_get_time();
        }
    }
}

