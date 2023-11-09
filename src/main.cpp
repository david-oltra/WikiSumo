#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <pinout.h>
#include <led.h>
#include <TB6612.h>
#include <QRE1113.h>
#include <startmode.h>
#include <labVl53l0x.h>

static char TAG[] = "MAIN";

extern "C" void app_main();

LED led_left(PIN_LED_L), led_front(PIN_LED_F), led_right(PIN_LED_R), led_back(PIN_LED_B);
TB6612 motores(PIN_MOTOR_A_1, PIN_MOTOR_A_2, PIN_MOTOR_A_PWM, PIN_MOTOR_B_1, PIN_MOTOR_B_2, PIN_MOTOR_B_PWM);
QRE1113 sensor_linea_i(PIN_QRE1113_L);
QRE1113 sensor_linea_d(PIN_QRE1113_R);
labVL53L0X sensor_distancia_i(I2C_NUM_0,PIN_VL53L0X_L);
labVL53L0X sensor_distancia_c(I2C_NUM_0,PIN_VL53L0X_C);
labVL53L0X sensor_distancia_d(I2C_NUM_0,PIN_VL53L0X_R);
START_MODE start_mode(PIN_MODE_ANALOG);


int64_t tiempo_espera_inicio = 5000;
int16_t tiempo_de_giro = 500;

static TaskHandle_t core_A = NULL;
static TaskHandle_t core_B = NULL;

uint8_t VL53L0X_I2C_ADDRESS[3] = {0x30, 0x31, 0x32};
gpio_num_t VL53L0X_XSHUT[3] = {PIN_VL53L0X_L, PIN_VL53L0X_C, PIN_VL53L0X_R};

void Init()
{
    led_left.Init(); led_front.Init(); led_right.Init(); led_back.Init();
    motores.Init();
    sensor_linea_i.Init(); sensor_linea_d.Init();
    start_mode.Init();

    sensor_distancia_i.Init(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,VL53L0X_I2C_ADDRESS[0]);
    gpio_set_level(VL53L0X_XSHUT[0],0);
    sensor_distancia_c.Init(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,VL53L0X_I2C_ADDRESS[1]);
    gpio_set_level(VL53L0X_XSHUT[1],0);
    sensor_distancia_d.Init(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,VL53L0X_I2C_ADDRESS[2]);
    gpio_set_level(VL53L0X_XSHUT[2],0);
    for (int8_t i = 0; i < 3; i++)
    {
        gpio_set_level(VL53L0X_XSHUT[i],1);
    }
    
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

void coreAThread(void *arg)
{
    ESP_LOGE(TAG, "Iniciando CORE A");

    int8_t start = gpio_get_level(PIN_BTN);
    while(!start)
    { 
        start = gpio_get_level(PIN_BTN);

        if (start_mode.read() == 0)
        {
            led_left.Update(LED::ON);
        }
        else if (start_mode.read() == 1)
        {
            led_front.Update(LED::ON);
        }
        else if (start_mode.read() == 2)
        {
            led_right.Update(LED::ON);
        }
        else if ( start_mode.read() == 3)
        {
            led_back.Update(LED::ON);
        } 
    };

    while(start)
    {
    led_left.Update(LED::BLINK_250ms);
    led_front.Update(LED::BLINK_250ms);
    led_right.Update(LED::BLINK_250ms);
    led_back.Update(LED::BLINK_250ms);
    }

    vTaskDelay(pdMS_TO_TICKS(tiempo_espera_inicio));    //5seg

    switch(start_mode.read())
    {  
        case 0:                             //LEFT
            motores.Update(TB6612::RIGHT);     
            vTaskDelay(pdMS_TO_TICKS(100));
        break;
        case 1:                             //FRONT
            motores.Update(TB6612::FWD);     
        break;
        case 2:                             //RIGHT
            motores.Update(TB6612::LEFT);     
            vTaskDelay(pdMS_TO_TICKS(100));
        break;
        case 3:                             //BACK
            motores.Update(TB6612::BACK);    
            vTaskDelay(pdMS_TO_TICKS(200));
        break;
    }

    while (true)
    {
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

