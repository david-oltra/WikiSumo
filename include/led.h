#ifndef __LED_H__
#define __LED_H__


class LED
{
    public:

        enum STATUS
        {
            OFF = 0,
            ON,
            BLINK_250ms,
            BLINK_500ms,
            BLINK_1s
        };

        LED(gpio_num_t _PIN_LED);

        void Init();

        void Update(enum STATUS st);
 
    private:
        gpio_num_t PIN_LED;


};

#endif