#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <pinout.h>
#include <leds.h>
#include <motores.h>
#include <sensorlinea.h>
#include <startmode.h>

static char TAG[] = "MAIN";

extern "C" void app_main();

LEDS leds(LED_I, LED_F, LED_D, LED_T);
MOTORES motores(MOTOR_A_1, MOTOR_A_2, MOTOR_B_1, MOTOR_B_2, MOTOR_A_PWM, MOTOR_B_PWM);
SENSOR_LINEA sensor_linea(LINEA_I, LINEA_D);

int64_t tiempo_espera_inicio = 5000;
int16_t tiempo_de_giro = 500;

static TaskHandle_t core_A = NULL;
static TaskHandle_t core_B = NULL;


void Init()
{
    leds.Init();
    motores.Init();
    sensor_linea.Init();
}

void CheckSensorLinea()
{
    int8_t sensor_i = gpio_get_level(LINEA_I);
    int8_t sensor_d = gpio_get_level(LINEA_D);
    
    if (sensor_i == 1)
    {
        motores.Update(0, 1, 1, 0); //GIRAR A LA DRCH
        vTaskDelay(pdMS_TO_TICKS(tiempo_de_giro));
    }
    else if (sensor_d == 1)
    {
        motores.Update(1, 0, 0, 1); //GIRAR A LA IZQ
        vTaskDelay(pdMS_TO_TICKS(tiempo_de_giro));
    }
    
}

void coreAThread(void *arg)
{
    ESP_LOGE(TAG, "Iniciando CORE A");

    int8_t start = gpio_get_level(BTN);
    while(!start)
    { 
        start = gpio_get_level(BTN);
        if (start_mode() == 0){
            leds.Update(1,0,0,0,1);
        }
        else if (start_mode() == 1)
        {
            leds.Update(0,1,0,0,1);
        }
        else if (start_mode() == 2)
        {
            leds.Update(0,0,1,0,1);
        }
        else if ( start_mode() == 3)
        {
            leds.Update(0,0,0,1,1);
        } 
    };

    while(start)
    {
    leds.Update(1,1,1,1,250);
    }

    vTaskDelay(pdMS_TO_TICKS(tiempo_espera_inicio));    //5seg

    switch(start_mode())
    {  
        case 0:                             //IZQ
            motores.Update(0, 1, 1, 0);     //GIRA DRCH
            vTaskDelay(pdMS_TO_TICKS(100));
        break;
        case 1:                             //FRONT
            motores.Update(0, 1, 0, 1);     //FWD
        break;
        case 2:                             //DRCH
            motores.Update(1, 0, 0, 1);     //GIRA IZQ
            vTaskDelay(pdMS_TO_TICKS(100));
        break;
        case 3:                             //TRAS
            motores.Update(1, 0, 0, 1);     //GIRA DRCH
            vTaskDelay(pdMS_TO_TICKS(200));
        break;
    }

    while (true)
    {
//        leds.Update(1, 0, 0, 0, 2000000);  //LED_I, LED_F, LED_D, LED_T, FREQ. (1seg=1000000)
        motores.Update(0, 1, 0, 1); //MOTOR_A_1, MOTOR_A_2, MOTOR_B_1, MOTOR_B_2
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

