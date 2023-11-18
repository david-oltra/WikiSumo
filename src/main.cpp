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
#include <VL53L0X.h>

#define I2C_PORT I2C_NUM_0

static char TAG[] = "MAIN";

extern "C" void app_main();

ROTARY rotary(PIN_ROTARY_S1, PIN_ROTARY_S2, PIN_ROTARY_KEY);
LED led_left(PIN_LED_L), led_front(PIN_LED_F), led_right(PIN_LED_R), led_back(PIN_LED_B);
TB6612 motores(PIN_MOTOR_A_1, PIN_MOTOR_A_2, PIN_MOTOR_A_PWM, PIN_MOTOR_B_1, PIN_MOTOR_B_2, PIN_MOTOR_B_PWM);
QRE1113 sensor_linea_i(PIN_QRE1113_L);
QRE1113 sensor_linea_d(PIN_QRE1113_R);
VL53L0X sensor_distancia_i(I2C_PORT, PIN_VL53L0X_L);
VL53L0X sensor_distancia_c(I2C_PORT, PIN_VL53L0X_C);
VL53L0X sensor_distancia_d(I2C_PORT, PIN_VL53L0X_R);

int64_t tiempo_espera_inicio = 5000;
int16_t tiempo_de_giro = 500;

static TaskHandle_t core_A = NULL;
static TaskHandle_t core_B = NULL;

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
    ESP_LOGI(TAG,"VL53L0X");

}

void Leds_update(LED::STATUS lft, LED::STATUS frnt, LED::STATUS rght, LED::STATUS bck)
{
    led_left.Update(lft);
    led_front.Update(frnt);
    led_right.Update(rght);
    led_back.Update(bck);
}


uint8_t VL53L0X_I2C_ADDRESS[3] = {0x30, 0x31, 0x32};
gpio_num_t VL53L0X_XSHUT[3] = {PIN_VL53L0X_L, PIN_VL53L0X_C, PIN_VL53L0X_R};

void Init_VL53L0X()
{
    for (uint8_t i=0; i<3; i++)
    {
        gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = (1ULL << VL53L0X_XSHUT[i]);
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&io_conf);
        gpio_set_level(VL53L0X_XSHUT[i],0);
    }
    vTaskDelay(pdMS_TO_TICKS(250));
    for (uint8_t i=0; i<3; i++)
    {
        gpio_set_level(VL53L0X_XSHUT[i],0);
    }
    sensor_distancia_i.i2cMasterInit(PIN_SDA,PIN_SCL);
    if (!sensor_distancia_i.init()) {
        ESP_LOGE(TAG, "Failed to initialize VL53L0X :(");
        while(1)
        {
            Leds_update(LED::BLINK_250ms, LED::OFF, LED::OFF, LED::OFF);
        }
    }
    sensor_distancia_i.setDeviceAddress(0x31);
    sensor_distancia_c.reset();
    sensor_distancia_d.reset();
    if (!sensor_distancia_c.init()) {
        ESP_LOGE(TAG, "Failed to initialize VL53L0X :(");
        while(1)
        {
            Leds_update(LED::OFF, LED::BLINK_250ms, LED::OFF, LED::OFF);
        }
    }
    sensor_distancia_c.setDeviceAddress(0x30);
    sensor_distancia_d.reset();
    if (!sensor_distancia_d.init()) {
        ESP_LOGE(TAG, "Failed to initialize VL53L0X :(");
        while(1)
        {
            Leds_update(LED::OFF, LED::OFF, LED::BLINK_250ms, LED::OFF);
        }
    }
}


uint16_t range_detect = 500; 
extern "C" bool CheckSensorTOF(VL53L0X sensor, uint i)
{
    uint16_t result_mm = 0;
    TickType_t tick_start = xTaskGetTickCount();
    bool res = sensor.read(&result_mm);
    TickType_t tick_end = xTaskGetTickCount();
    int took_ms = ((int)tick_end - tick_start) / portTICK_PERIOD_MS;
    if (res)
    {
        ESP_LOGI(TAG, "Sensor %u Range: %d [mm] took %d [ms]", i, (int)result_mm, took_ms);
    }            
    else
    ESP_LOGE(TAG, "Failed to measure :(");

    if ((int)result_mm < range_detect)
    {
        return 1;
    }
    else {
        return 0;
    }
}


bool CheckSensorLinea(QRE1113 sensor_linea)
{
    return gpio_get_level(sensor_linea.PinNum());
}

int8_t STATUS_TOF = 0;
int8_t start = 0;
int8_t selected_start_mode;
void coreAThread(void *arg)
{
    ESP_LOGE(TAG, "Iniciando CORE A");

    while(rotary.Key())
    { 
//        ESP_LOGI(TAG,"CORE_A start: %u",rotary.Key());
        uint8_t status_rotary = rotary.CheckEncoder();
//        printf("case: %u \n", status_rotary);

        switch (status_rotary)
        {
        case 0:
            Leds_update(LED::ON, LED::OFF, LED::OFF, LED::OFF);
        //    led_left.Update(LED::ON);
        //    led_front.Update(LED::OFF);
        //    led_right.Update(LED::OFF);
        //    led_back.Update(LED::OFF);
            selected_start_mode = 0;
        break;
        case 1:
            Leds_update(LED::OFF, LED::ON, LED::OFF, LED::OFF);
            selected_start_mode = 1;
        break;
        case 2:
            Leds_update(LED::OFF, LED::OFF, LED::ON, LED::OFF);
            selected_start_mode = 2;
        break;
        case 3:
            Leds_update(LED::OFF, LED::OFF, LED::OFF, LED::ON);
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
    start = 1;

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
            motores.Update(TB6612::RIGHT);    
            vTaskDelay(pdMS_TO_TICKS(500));
        break;
        default:
        break;
    }

    while (true)
    {
    //    ESP_LOGI(TAG,"CORE_A: SEARCHING");
        if (!STATUS_TOF) 
        {
            motores.Update(TB6612::FWD);
            Leds_update(LED::OFF, LED::OFF, LED::OFF, LED::OFF);
        } 
        if (CheckSensorLinea(sensor_linea_i))
        {
            motores.Update(TB6612::RIGHT);
            Leds_update(LED::OFF, LED::OFF, LED::ON, LED::OFF);
            vTaskDelay(pdMS_TO_TICKS(tiempo_de_giro));
        }
        else if (CheckSensorLinea(sensor_linea_d))
        {
            motores.Update(TB6612::LEFT);
            Leds_update(LED::ON, LED::OFF, LED::OFF, LED::OFF);
            vTaskDelay(pdMS_TO_TICKS(tiempo_de_giro));
        }
    }
}


void coreBThread(void *arg)
{
    ESP_LOGE(TAG, "Iniciando CORE B");    
    while(1)
    {
        while(start)
        {
            if (CheckSensorTOF(sensor_distancia_c,1))
            {
                STATUS_TOF = 1;
                motores.Update(TB6612::FWD);
                Leds_update(LED::OFF, LED::ON, LED::OFF, LED::OFF);
            }
            else if (CheckSensorTOF(sensor_distancia_i,0))
            {
                STATUS_TOF = 1;
                motores.Update(TB6612::LEFT);
                Leds_update(LED::ON, LED::OFF, LED::OFF, LED::OFF);
            }
            else if (CheckSensorTOF(sensor_distancia_d,2))
            {
                STATUS_TOF = 1;
                motores.Update(TB6612::RIGHT);
                Leds_update(LED::OFF, LED::OFF, LED::ON, LED::OFF);
            }
            else{
                STATUS_TOF = 0;
            }

        }
    }

}


void i2c_scanner()
{
  //    CheckSensorTOF(sensor_distancia_i);
        int i;
        esp_err_t espRc;
        printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
        printf("00:         ");
        for (i=3; i< 0x78; i++) {
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
            i2c_master_stop(cmd);

            espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
            if (i%16 == 0) {
                printf("\n%.2x:", i);
            }
            if (espRc == 0) {
                printf(" %.2x", i);
            } else {
                printf(" --");
            }
            //ESP_LOGD(tag, "i=%d, rc=%d (0x%x)", i, espRc, espRc);
            i2c_cmd_link_delete(cmd);
	}
	printf("\n");
	vTaskDelete(NULL);
}


void app_main()
{   
    ESP_LOGE(TAG, "Iniciando software");
    Init();
    Init_VL53L0X();

    while(0)
    {
    //    CheckSensorTOF(sensor_distancia_i,0);
    //    CheckSensorTOF(sensor_distancia_c,1);
    //    CheckSensorTOF(sensor_distancia_d,2);

        i2c_scanner();
    }



    xTaskCreatePinnedToCore(coreBThread, "core_B", 4096, NULL, 9, &core_B, 1);    
    xTaskCreatePinnedToCore(coreAThread, "core_A", 4096, NULL, 10, &core_A, 0);
    
}

