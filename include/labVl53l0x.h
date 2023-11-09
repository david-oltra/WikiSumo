#ifndef __LABVL56L0X_H__
#define __LABVL56L0X_H__

#include "vl53l0x_api.h"
#include "vl53l0x_def.h"
#include "vl53l0x_platform.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define NUM_TOF_SENSORS 3
#define NUM_LINE_SENSORS 2


class labVL53L0X
{
public:
    labVL53L0X(i2c_port_t i2c_port = I2C_NUM_0, gpio_num_t gpio_xshut = GPIO_NUM_MAX,
               gpio_num_t gpio_gpio1 = GPIO_NUM_MAX)
        : i2c_port(i2c_port), gpio_xshut(gpio_xshut), gpio_gpio1(gpio_gpio1)
    {
        vSemaphoreCreateBinary(xSemaphore);
    }

    bool Init(VL53L0X_DeviceModes devMode, uint8_t VL53L0X_I2C_ADDRESS);

    bool setThresholds(uint32_t rate, uint32_t threshold);
    
    bool setSPads(uint32_t padCount, uint32_t isAperture);


    /* Reading functions */
    bool readSingleWithPolling(uint16_t *pRangeMilliMeter);
    bool readContiniousLastData(uint16_t *pRangeMilliMeter);

    void i2cMasterInit(gpio_num_t pin_sda = GPIO_NUM_21,
                       gpio_num_t pin_scl = GPIO_NUM_22, uint32_t freq = 400000);

    bool setTimingBudget(uint32_t TimingBudgetMicroSeconds);

private : 
    i2c_port_t i2c_port;
    gpio_num_t gpio_xshut;
    gpio_num_t gpio_gpio1;
    VL53L0X_Dev_t vl53l0x_dev;
    volatile SemaphoreHandle_t xSemaphore = NULL;
    int32_t TimingBudgetMicroSeconds;

    VL53L0X_Error init_vl53l0x(VL53L0X_DeviceModes devMode);
    VL53L0X_Error print_pal_error(VL53L0X_Error status,
                                         const char *method);

    uint32_t tofRateValue[NUM_TOF_SENSORS] = {70,70,80};
    uint32_t tofThresholdValue[NUM_TOF_SENSORS] = {95,95,98};

    uint32_t lastTofValues[NUM_TOF_SENSORS] = {0,0,0};

    /* Critical sections */
    uint16_t tofData[NUM_TOF_SENSORS];
    uint32_t lineData[NUM_LINE_SENSORS];
};

#endif //__LABVL56L0X_H__