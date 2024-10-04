#include <version.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <nvs_flash.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_http_client.h>
#include <esp_wifi.h>
#include <esp_https_ota.h>
#include <pinout.h>
#include <led.h>
#include <TB6612.h>
#include <QRE1113.h>
#include <VL53L0X.h>
#include <driver/adc.h>
#include <espnow.h>

#include "secret.h"

#define ENABLE_OTA
#ifdef ENABLE_OTA
extern "C" void wifi_init_sta();
#endif
#ifndef WIFI_PWD  
    #define WIFI_PWD "********"
#endif
#ifndef WIFI_SSID
    #define WIFI_SSID "********"
#endif


#define I2C_PORT I2C_NUM_0

static char TAG[] = "MAIN";

extern "C" void app_main();

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

esp_err_t EnableWifi(void)
{
#ifdef ENABLE_OTA
    wifi_init_sta();
#endif

    return ESP_OK;
}

esp_err_t Leds_update(LED::STATUS lft, LED::STATUS frnt, LED::STATUS rght, LED::STATUS bck)
{
    led_left.Update(lft);
    led_front.Update(frnt);
    led_right.Update(rght);
    led_back.Update(bck);

    return ESP_OK;
}

esp_err_t updateOTA(void)
{
    char newVersionFW[50];
    memset(newVersionFW, 0, 50);

    esp_http_client_config_t configFW = {
        .url = "http://192.168.0.16:8000/VERSION.html",
        .user_data = newVersionFW,        // Pass address of local buffer to get response
    };
    esp_http_client_handle_t client = esp_http_client_init(&configFW);
    esp_http_client_open(client,0);
    esp_http_client_fetch_headers(client);
    esp_http_client_read(client, newVersionFW, esp_http_client_get_content_length(client));

    // ESP_LOGE(TAG,"Received html: %s", newVersionFW);
    uint64_t value;
    value = atoll(newVersionFW);
    ESP_LOGI(TAG,"Received Version: %llu", value);
    ESP_LOGI(TAG,"Actual Version: %llu", actualVersion);

    esp_http_client_close(client);
    esp_http_client_cleanup(client);

   

    if (value > actualVersion)
    {
        Leds_update(LED::OFF, LED::OFF, LED::OFF, LED::OFF);
        ESP_LOGE(TAG, "NEW VERSION!!!");
    
        ESP_LOGI(TAG, "Starting OTA task");
        esp_http_client_config_t config = {
            .url = "http://192.168.0.16:8000/firmware.bin?",
            .keep_alive_enable = true,
        };

        esp_https_ota_config_t ota_config = {
            .http_config = &config,
        };
        ESP_LOGI(TAG, "Attempting to download update from %s", config.url);
        esp_err_t ret = esp_https_ota(&ota_config);
        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "OTA Succeed, Rebooting...");
            esp_restart();
        }
        else
        {
            ESP_LOGE(TAG, "Firmware upgrade failed");
        }
    }

    return ESP_OK;
}


uint8_t VL53L0X_I2C_ADDRESS[3] = {0x30, 0x31, 0x32};
gpio_num_t VL53L0X_XSHUT[3] = {PIN_VL53L0X_L, PIN_VL53L0X_C, PIN_VL53L0X_R};

esp_err_t Init_VL53L0X(void)
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
        ESP_LOGE(TAG, "Failed to initialize VL53L0X I :(");
        while(1)
        {
            Leds_update(LED::BLINK_250ms, LED::OFF, LED::OFF, LED::OFF);
        }
    }
    sensor_distancia_i.setDeviceAddress(0x31);
    sensor_distancia_c.reset();
    sensor_distancia_d.reset();
    vTaskDelay(pdMS_TO_TICKS(250));
    if (!sensor_distancia_c.init()) {
        ESP_LOGE(TAG, "Failed to initialize VL53L0X C :(");
        while(1)
        {
            Leds_update(LED::OFF, LED::BLINK_250ms, LED::OFF, LED::OFF);
        }
    }
    sensor_distancia_c.setDeviceAddress(0x30);   
    sensor_distancia_d.reset();
    if (!sensor_distancia_d.init()) {
        ESP_LOGE(TAG, "Failed to initialize VL53L0X D :(");
        while(1)
        {
            Leds_update(LED::OFF, LED::OFF, LED::BLINK_250ms, LED::OFF);
        }
    }

    return ESP_OK;
}

esp_err_t Init_Key(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << PIN_KEY);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    return ESP_OK;
}


uint16_t range_detect = 100; 
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


bool STATUS_TOF = 0;
bool start = 0;
bool start_core_b = 0;
int8_t selected_start_mode = 0;
int ref_time = 0;
int key_time = 0;
bool key_status = 0;
bool key_push = 0;
void coreAThread(void *arg)
{
    ESP_LOGE(TAG, "Iniciando CORE A");
    
    while(1)
    {
        key_status = gpio_get_level(PIN_KEY) | !remote_data.sw;

        while(key_status)
        {
            if(!key_push)
                {
                    ref_time = esp_timer_get_time();
                }
            key_time = esp_timer_get_time();
            if (key_time - ref_time > 3000000)
                {
                if(selected_start_mode == 0){led_left.Update(LED::ON);}   else {led_left.Update(LED::BLINK_250ms);}
                if(selected_start_mode == 1){led_front.Update(LED::ON);}   else {led_front.Update(LED::BLINK_250ms);}
                if(selected_start_mode == 2){led_right.Update(LED::ON);}   else {led_right.Update(LED::BLINK_250ms);}
                if(selected_start_mode == 3){led_back.Update(LED::ON);}   else {led_back.Update(LED::BLINK_250ms);}
                start = 1;
                }
            key_push = 1;
            key_status = gpio_get_level(PIN_KEY) | !remote_data.sw;
        }

        if (key_push && (key_time - ref_time < 3000000))
        {
            key_push = 0;
            selected_start_mode ++;
            if (selected_start_mode > 3){selected_start_mode = 0;};
            switch (selected_start_mode)
            {
            case 0:
                Leds_update(LED::ON, LED::OFF, LED::OFF, LED::OFF);
            break;
            case 1:
                Leds_update(LED::OFF, LED::ON, LED::OFF, LED::OFF);
            break;
            case 2:
                Leds_update(LED::OFF, LED::OFF, LED::ON, LED::OFF);
            break;
            case 3:
                Leds_update(LED::OFF, LED::OFF, LED::OFF, LED::ON);
            break;
            default:
            break;
            }
            ESP_LOGE(TAG, "selected_start_mode: %u", selected_start_mode);

        }
    
        while (start)
        {
            ESP_LOGE(TAG, "START!");
            vTaskDelay(pdMS_TO_TICKS(tiempo_espera_inicio));    //5seg
            start_core_b = 1;
            ESP_LOGE(TAG, "GO!");
            switch(selected_start_mode)
            {  
                case 0:                             //LEFT
                    motores.Update(TB6612::LEFT);     
                    vTaskDelay(pdMS_TO_TICKS(tiempo_de_giro));
                break;
                case 1:                             //FRONT
                    motores.Update(TB6612::FWD);     
                break;
                case 2:                             //RIGHT
                    motores.Update(TB6612::RIGHT);     
                    vTaskDelay(pdMS_TO_TICKS(tiempo_de_giro));
                break;
                case 3:                             //BACK
                    motores.Update(TB6612::RIGHT);    
                    vTaskDelay(pdMS_TO_TICKS(tiempo_de_giro));
                break;
                default:
                break;
            }

            while(remote_data.sw)
            {
                if (!STATUS_TOF)
                {
                    motores.Update(TB6612::FWD);
                    Leds_update(LED::OFF, LED::OFF, LED::OFF, LED::OFF);
                }
                uint16_t adc_sensor_i = adc1_get_raw(ADC1_CHANNEL_0); //85
                uint16_t adc_sensor_d = adc1_get_raw(ADC1_CHANNEL_3); //1800
                printf("SensorI: %u \t SensorD: %u\n", adc_sensor_i, adc_sensor_d);
                if (adc_sensor_i < 75)
                {
                    motores.Update(TB6612::LEFT);
                    Leds_update(LED::OFF, LED::OFF, LED::ON, LED::OFF);
                    vTaskDelay(pdMS_TO_TICKS(tiempo_de_giro));
                }
                if (adc_sensor_d < 1800)
                {
                    motores.Update(TB6612::RIGHT);
                    Leds_update(LED::ON, LED::OFF, LED::OFF, LED::OFF);
                    vTaskDelay(pdMS_TO_TICKS(tiempo_de_giro));
                }                
            }
            esp_restart();
        }
    }
}


void coreBThread(void *arg)
{
    ESP_LOGE(TAG, "Iniciando CORE B");    
    while(1)
    {
        while(start_core_b)
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
 //           ESP_LOGE(TAG, "tof_C: %u\t tof_i: %u\t tof_d: %u", CheckSensorTOF(sensor_distancia_c,1), CheckSensorTOF(sensor_distancia_i,0), CheckSensorTOF(sensor_distancia_d,2));
        }
    }

}

void app_main()
{  
    ESP_LOGW(TAG, "Iniciando software");
    ESP_LOGW(TAG, "NVS_FLASH INIT");
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_LOGW(TAG, "NVS_FLASH INIT OK");
    ESP_LOGW(TAG, "WIFI ENABLE");
    while(EnableWifi() != ESP_OK);
    ESP_LOGW(TAG, "WIFI ENABLE OK");
    ESP_LOGW(TAG, "WIFI CONNECT");
    if (esp_wifi_connect() == ESP_OK)
    {
        ESP_LOGW(TAG, "WIFI CONNECTED");
        ESP_LOGW(TAG, "OTA CHECK OK");
        while(updateOTA() != ESP_OK);
        ESP_LOGW(TAG, "OTA CHECK OK");
        ESP_LOGW(TAG, "WIFI DISCONNECT");
        ESP_ERROR_CHECK(esp_wifi_disconnect());
        ESP_ERROR_CHECK(esp_wifi_stop());
        ESP_LOGW(TAG, "WIFI DISCONNECTED");
   
    }
    else
    {
        ESP_LOGW(TAG, "WIFI NOT CONNECTED");
    }
    ESP_LOGW(TAG, "ESP_NOW INIT");
    ESP_ERROR_CHECK(init_esp_now());
    ESP_ERROR_CHECK(esp_wifi_disconnect());
    ESP_LOGW(TAG, "ESP_NOW INIT OK");
    ESP_LOGW(TAG,"LEDS INIT");
    led_left.Init(); 
    led_front.Init(); 
    led_right.Init(); 
    led_back.Init();
    ESP_LOGW(TAG,"LEDS INIT OK");
    ESP_LOGW(TAG,"TB6612 INIT");
    motores.Init();
    ESP_LOGW(TAG,"TB6612 INIT OK");
    ESP_LOGW(TAG,"QRE1113 INIT");
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_11);
    ESP_LOGI(TAG,"ADC_WIDTH_BIT_12|\tADC1_CHANNEL_0,ADC_ATTEN_DB_11");
    ESP_LOGW(TAG,"QRE1113 INIT OK");
    ESP_LOGW(TAG,"VL53L0X INIT");
    while(Init_VL53L0X() != ESP_OK);
    ESP_LOGW(TAG,"VL53L0X INIT OK");
    ESP_LOGW(TAG,"VL53L0X INIT");
    while(Init_Key() != ESP_OK);
    ESP_LOGW(TAG,"VL53L0X INIT OK");

    xTaskCreatePinnedToCore(coreBThread, "core_B", 4096, NULL, 9, &core_B, 1);    
    xTaskCreatePinnedToCore(coreAThread, "core_A", 4096, NULL, 10, &core_A, 0);
    
}

