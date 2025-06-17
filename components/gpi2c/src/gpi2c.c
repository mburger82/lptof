#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "../gpi2c.h"

#define TAG "GPI2C"

static SemaphoreHandle_t i2cMutex = NULL;
int i2c_master_port = 1;

esp_err_t gpi2c_readRegister(uint8_t addr, uint8_t reg, uint8_t * data, uint16_t len) {
    esp_err_t err = ESP_ERR_TIMEOUT;
    uint8_t readReg[1] = {reg};
    // ESP_LOGI(TAG, "Read Register: addr: %02X - reg: %02X - n: %i ", addr, reg, len);
    if(xSemaphoreTake(i2cMutex, 50/portTICK_PERIOD_MS) == pdTRUE) {
        // ESP_LOGI(TAG, "Mutex ok");
        err = i2c_master_write_read_device(i2c_master_port, addr, readReg, 1, data, len, portMAX_DELAY);
        xSemaphoreGive(i2cMutex);
    } else {
        ESP_LOGE(TAG, "Semaphore error");
    }
    return err;
}

esp_err_t gpi2c_writeRegister(uint8_t addr, uint8_t reg, uint8_t * data, uint16_t len) {
    esp_err_t err = ESP_ERR_TIMEOUT;
    // ESP_LOGI(TAG, "Write Register: addr: %02X - reg: %02X - n: %i ", addr, reg, len);
    if(xSemaphoreTake(i2cMutex, 50/portTICK_PERIOD_MS) == pdTRUE) {
        // ESP_LOGI(TAG, "Mutex ok");
        uint8_t senddata[len+1];
        senddata[0] = reg;
        for(int i = 1; i <= len;i++) {
            senddata[i] = data[i-1];
        }
        err = i2c_master_write_to_device(i2c_master_port, addr, &senddata[0], len+1, portMAX_DELAY);
        xSemaphoreGive(i2cMutex);
    } else {
        ESP_LOGE(TAG, "Semaphore error");
    }
    return err;
}

esp_err_t gpi2c_writeData(uint8_t addr, uint8_t * data, uint16_t len) {
    esp_err_t err = ESP_ERR_TIMEOUT;
    if(xSemaphoreTake(i2cMutex, 50/portTICK_PERIOD_MS)) {
        // ESP_LOGI(TAG, "Mutex ok");
        err = i2c_master_write_to_device(i2c_master_port, addr, &data[0], len, portMAX_DELAY);
        xSemaphoreGive(i2cMutex);
    } else {
        ESP_LOGE(TAG, "Semaphore error");
    }
    return err;
}

void gpi2c_init(int pinI2C_SDA, int pinI2C_SCL, uint32_t frequency) {
    if(i2cMutex == NULL) {
        ESP_LOGI(TAG, "Init GPI2C...");
        ESP_LOGI(TAG, "I2C Config: SDA:%i - SCL:%i - Speed:%iHz", pinI2C_SDA, pinI2C_SCL, (int)frequency);
        i2cMutex = xSemaphoreCreateMutex();
        i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = pinI2C_SDA,
            .scl_io_num = pinI2C_SCL,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = frequency,
        };
        i2c_param_config(i2c_master_port, &conf);
        ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0));

        ESP_LOGI(TAG, "Init GPI2C done");
    } else {
        ESP_LOGW(TAG, "GPI2C already initialized");
    }
}