#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <pinout.h>
#include <leds.h>
#include <esp_timer.h>
#include <esp_log.h>


LEDS::LEDS(gpio_num_t _led_i,gpio_num_t _led_f,gpio_num_t _led_d,gpio_num_t _led_t)
{
    led_i = _led_i;
    led_f = _led_f;
    led_d = _led_d;
    led_t = _led_t;
}

void LEDS::Init()
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << LED_I) | (1ULL << LED_F) | (1ULL << LED_D) | (1ULL << LED_T);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    
    gpio_set_level(led_i,1);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(led_f,1);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(led_d,1);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(led_t,1);
    vTaskDelay(pdMS_TO_TICKS(500));
    
}

uint8_t led_status = 0;
int64_t timer_ref_led = esp_timer_get_time();
int64_t timer_led;

void LEDS::Update(int8_t _led_i, int8_t _led_f, int8_t _led_d, int8_t _led_t, int64_t _freq)
{
    if (_freq <= 1)
    {
        gpio_set_level(LED_I,_led_i);
        gpio_set_level(LED_F,_led_f);
        gpio_set_level(LED_D,_led_d);
        gpio_set_level(LED_T,_led_t);
    }
    else
    {
        timer_led = esp_timer_get_time() - timer_ref_led;
        if (timer_led > _freq)
        {
            timer_led = esp_timer_get_time() - timer_ref_led;
            led_status = !led_status;
            if (_led_i == 1) { gpio_set_level(LED_I, led_status); }
            if (_led_f == 1) { gpio_set_level(LED_F, led_status); }
            if (_led_d == 1) { gpio_set_level(LED_D, led_status); }
            if (_led_t == 1) { gpio_set_level(LED_T, led_status); }
            timer_ref_led = esp_timer_get_time();
        }
        

    }
}

