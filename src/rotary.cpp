#include <rotary.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <esp_timer.h>

#define MAX_OPTIONS 3

ROTARY::ROTARY(gpio_num_t _PIN_S1, gpio_num_t _PIN_S2, gpio_num_t _PIN_KEY)
{
    PIN_S1 = _PIN_S1;
    PIN_S2 = _PIN_S2;
    PIN_KEY = _PIN_KEY;
}

void ROTARY::Init()
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << PIN_S1 | 1ULL << PIN_S2 | 1ULL << PIN_KEY);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

}

int64_t timer;
int64_t timer_ref;
uint8_t count = 0;

uint8_t ROTARY::CheckEncoder()
{
    if (gpio_get_level(PIN_S1) != gpio_get_level(PIN_S2))
    {
        if (gpio_get_level(PIN_S1)){count++;}
        else {count--;}
        timer_ref = esp_timer_get_time();
        timer = esp_timer_get_time() - timer_ref;
        while(timer < 500000)
        {
            timer = esp_timer_get_time() - timer_ref;
        }
        timer_ref = esp_timer_get_time();
    }
    if (count>MAX_OPTIONS+1){count=MAX_OPTIONS;}
    else if (count>MAX_OPTIONS){count=0;}

    return count;
}

bool ROTARY::Key()
{
    bool key_status = gpio_get_level(PIN_KEY);
    return key_status;
}