#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <pinout.h>
#include <rotary.h>
#include <led.h>
#include <TB6612.h>
#include <QRE1113.h>

static char TAG[] = "MAIN";

extern "C" void app_main();

ROTARY rotary(PIN_ROTARY_S1, PIN_ROTARY_S2, PIN_ROTARY_KEY);
LED led_left(PIN_LED_L), led_front(PIN_LED_F), led_right(PIN_LED_R), led_back(PIN_LED_B);
TB6612 motores(PIN_MOTOR_A_1, PIN_MOTOR_A_2, PIN_MOTOR_A_PWM, PIN_MOTOR_B_1, PIN_MOTOR_B_2, PIN_MOTOR_B_PWM);
QRE1113 sensor_linea_i(PIN_QRE1113_L);
QRE1113 sensor_linea_d(PIN_QRE1113_R);

//dVL53L0X sensor_distancia(I2C_NUM_0, PIN_SDA, PIN_SCL, PIN_VL53L0X_C, 0X32);

int64_t tiempo_espera_inicio = 5000;
int16_t tiempo_de_giro = 500;

static TaskHandle_t core_A = NULL;
static TaskHandle_t core_B = NULL;

uint8_t VL53L0X_I2C_ADDRESS[3] = {0x30, 0x31, 0x32};
gpio_num_t VL53L0X_XSHUT[3] = {PIN_VL53L0X_L, PIN_VL53L0X_C, PIN_VL53L0X_R};

void Init()
{
    ESP_LOGI(TAG,"INIT");
    ESP_LOGI(TAG,"ROTARY ENCODER");
    rotary.Init();
    ESP_LOGI(TAG,"LEDS");
    led_left.Init(); 
    vTaskDelay(pdMS_TO_TICKS(50));
    led_front.Init(); 
    vTaskDelay(pdMS_TO_TICKS(50));
    led_right.Init(); 
    vTaskDelay(pdMS_TO_TICKS(50));
    led_back.Init();
    ESP_LOGI(TAG,"MOTORES");
    motores.Init();
    ESP_LOGI(TAG,"LINEAS");
    sensor_linea_i.Init(); 
    sensor_linea_d.Init();
    ESP_LOGI(TAG,"START_MODE");
    //ESP_LOGI(TAG,"VL53L0X");
}


void CheckSensorLinea()
{
    int8_t sensor_l = gpio_get_level(PIN_QRE1113_L);
    int8_t sensor_r = gpio_get_level(PIN_QRE1113_R);
    
    if (sensor_l == 1)
    {
        motores.Update(TB6612::RIGHT);
        vTaskDelay(pdMS_TO_TICKS(tiempo_de_giro));
    }
    else if (sensor_r == 1)
    {
        motores.Update(TB6612::LEFT);
        vTaskDelay(pdMS_TO_TICKS(tiempo_de_giro));
    }
    
}

int8_t selected_start_mode;
void coreAThread(void *arg)
{
    ESP_LOGE(TAG, "Iniciando CORE A");

    while(rotary.Key())
    { 
        ESP_LOGI(TAG,"CORE_A start: %u",rotary.Key());

        switch (rotary.CheckEncoder())
        {
        case 0:
            led_left.Update(LED::ON);
            led_front.Update(LED::OFF);
            led_right.Update(LED::OFF);
            led_back.Update(LED::OFF);
            selected_start_mode = 0;
        break;
        case 1:
            led_left.Update(LED::OFF);
            led_front.Update(LED::ON);
            led_right.Update(LED::OFF);
            led_back.Update(LED::OFF);
            selected_start_mode = 1;
        break;
        case 2:
            led_left.Update(LED::OFF);
            led_front.Update(LED::OFF);
            led_right.Update(LED::ON);
            led_back.Update(LED::OFF);
            selected_start_mode = 2;
        break;
        case 3:
            led_left.Update(LED::OFF);
            led_front.Update(LED::OFF);
            led_right.Update(LED::OFF);
            led_back.Update(LED::ON);
            selected_start_mode = 3;
        break;
        default:
        break;
        }
    };

    while(!rotary.Key())
    {
        if(selected_start_mode == 0){led_left.Update(LED::ON);}   else {led_left.Update(LED::BLINK_250ms);}
        if(selected_start_mode == 1){led_front.Update(LED::ON);}   else {led_front.Update(LED::BLINK_250ms);}
        if(selected_start_mode == 2){led_right.Update(LED::ON);}   else {led_right.Update(LED::BLINK_250ms);}
        if(selected_start_mode == 3){led_back.Update(LED::ON);}   else {led_back.Update(LED::BLINK_250ms);}
    }

    vTaskDelay(pdMS_TO_TICKS(tiempo_espera_inicio));    //5seg

    switch(selected_start_mode)
    {  
        case 0:                             //LEFT
            motores.Update(TB6612::RIGHT);     
            vTaskDelay(pdMS_TO_TICKS(250));
        break;
        case 1:                             //FRONT
            motores.Update(TB6612::FWD);     
        break;
        case 2:                             //RIGHT
            motores.Update(TB6612::LEFT);     
            vTaskDelay(pdMS_TO_TICKS(250));
        break;
        case 3:                             //BACK
            motores.Update(TB6612::BACK);    
            vTaskDelay(pdMS_TO_TICKS(500));
        break;
        default:
        break;
    }

    while (true)
    {
        ESP_LOGI(TAG,"CORE_A: SEARCHING");
        motores.Update(TB6612::FWD); 
        CheckSensorLinea();
    }
}

void coreBThread(void *arg)
{
    ESP_LOGE(TAG, "Iniciando CORE B");
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(2000));
        ESP_LOGI(TAG, "Core B");


    }
}


void app_main() 
{   
    ESP_LOGE(TAG, "Iniciando software");
    Init();


    xTaskCreatePinnedToCore(coreBThread, "core_B", 4096, NULL, 9, &core_B, 1);    
    xTaskCreatePinnedToCore(coreAThread, "core_A", 4096, NULL, 10, &core_A, 0);
}


