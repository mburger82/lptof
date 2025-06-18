#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <assert.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include <fcntl.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"

#include "esp_vfs_fat.h"
#include "cmd_system.h"

#include "gpi2c.h"
#include "tmf8828_app.h"
#include "tmf8820.h"




#define TAG "tmf8820"

uint16_t tofconf_updatetime = 100;
uint16_t tofconf_kilo_iterations = 25;
uint8_t tofconf_spad = 1;
uint16_t tofconf_threshold_low_mm = 100;
uint16_t tofconf_threshold_high_mm = 2000;


// #define TMF8820_ADDRESS 0x41

// void writeTMF8820Register(uint8_t reg, uint8_t data) {
//     gpi2c_writeRegister(TMF8820_ADDRESS, reg, (uint8_t*)&data, 1);
// }
// void readTMF8820Register(uint8_t reg, uint8_t* data) {
//     ESP_LOGI(TAG, "i2c return: %i", gpi2c_readRegister(TMF8820_ADDRESS, reg, data, 1));
// }

float measuredDistance = 0;
SemaphoreHandle_t mutex_getDistanceData;

void tmf8820task(void* param) {
    mutex_getDistanceData = xSemaphoreCreateMutex();
    tmf882x_setConfig(tofconf_updatetime, tofconf_kilo_iterations, tofconf_spad, tofconf_threshold_low_mm, tofconf_threshold_high_mm);
    
    setupFn(0, 9600, 400000);
    vTaskDelay(50/portTICK_PERIOD_MS);
    loopFn('d');
    vTaskDelay(1000/portTICK_PERIOD_MS);
    loopFn('e');
    vTaskDelay(2000/portTICK_PERIOD_MS);
    loopFn('#');
    vTaskDelay(50/portTICK_PERIOD_MS);
    loopFn('e');
    vTaskDelay(2000/portTICK_PERIOD_MS);
    // loopFn('h');
    // vTaskDelay(200/portTICK_PERIOD_MS);
    loopFn('c');
    vTaskDelay(20/portTICK_PERIOD_MS);
    loopFn('m');
    
    char ch;
    for(;;) {
        ch = 0;
        // loopFn('m');
        // vTaskDelay(1);
        loopFn(0);
        // loopFn('s');
        // Work with resultdata
        // ESP_LOGI(TAG, "Found Results: %i", resultdata.numberOfValidResults);
        float meandistance = 0;
        uint8_t confidentresults = 0;
        if(resultdata.numberOfValidResults > 0) {
            // printf("\n confidences: ");
            for(int i = 0; i < MAX_RESULT_VALUES; i++) {
                if((resultdata.confidence[i] > 10) && (resultdata.distance[i] > 0)) {
                    meandistance += resultdata.distance[i];
                    // printf("%i, ", resultdata.confidence[i]);
                    confidentresults++;
                }
            }
            // printf("\n");
            meandistance /= confidentresults;
        }
        xSemaphoreTake(mutex_getDistanceData, portMAX_DELAY);
        if(meandistance > 0) {
            measuredDistance = meandistance;            
        } else {
            measuredDistance = 0;
        }
        xSemaphoreGive(mutex_getDistanceData);
        // ESP_LOGI(TAG, "Mean measured distance: %f", meandistance);
        vTaskDelay(tofconf_updatetime/portTICK_PERIOD_MS);
    }
}

bool tmf8820_detected() {
    bool result = false;
    xSemaphoreTake(mutex_getDistanceData, portMAX_DELAY);
    if(measuredDistance > 0) {
        result = true;
    }
    xSemaphoreGive(mutex_getDistanceData);
    return result;
}
uint16_t tmf8820_distance() {
    uint16_t result = 0;
    xSemaphoreTake(mutex_getDistanceData, portMAX_DELAY);
    result = measuredDistance;
    xSemaphoreGive(mutex_getDistanceData);
    return result;
}


void initTMF8820(uint16_t updatetime, uint16_t kilo_iterations, uint8_t spad, uint16_t threshold_low_mm, uint16_t threshold_high_mm) {
    tofconf_updatetime = updatetime;
    tofconf_kilo_iterations = kilo_iterations;
    tofconf_spad = spad;
    tofconf_threshold_low_mm = threshold_low_mm;
    tofconf_threshold_high_mm = threshold_high_mm;
    xTaskCreate(tmf8820task, "taskTMF8820", 2048*2, NULL, 10, NULL);
}