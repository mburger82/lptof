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

#include "buttonhandler.h"

#define BUTTON_GPIO_MASK  ((1ULL<<BUTTON_GPIO_REC) | (1ULL<<BUTTON_GPIO_MODE))
#define BUTTON_MAX          2

typedef struct {
    uint8_t gpio;
    uint8_t count;
    buttonstate_t state;
} button_t;

button_t buttondata[BUTTON_MAX];

SemaphoreHandle_t mButtonMutex;

void buttonTask(void* param) {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = BUTTON_GPIO_MASK;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    for(int i = 0; i < BUTTON_MAX; i++) {
        buttondata[i].count = 0;
        buttondata[i].state = BUTTON_NOT_PRESSED;
    }
    buttondata[BUTTON_REC].gpio = BUTTON_GPIO_REC;
    buttondata[BUTTON_MODE].gpio = BUTTON_GPIO_MODE;

    for(;;) {
        xSemaphoreTake(mButtonMutex, portMAX_DELAY);
        for(int i = 0; i < BUTTON_MAX; i++) {
            if(gpio_get_level(buttondata[i].gpio) == 0) {
                buttondata[i].count++;
            } else {
                if(buttondata[i].count > (BUTTON_PRESS_LONG_TIME_MS/BUTTONTASK_UPDATE_TIME_MS)) {
                    // Long press detected
                    buttondata[i].state = BUTTON_PRESSED_LONG;
                } else if(buttondata[i].count > (BUTTON_PRESS_SHORT_TIME_MS/BUTTONTASK_UPDATE_TIME_MS)) {
                    // Short press detectd
                    buttondata[i].state = BUTTON_PRESSED_SHORT;
                } else {
                    // No press detected                    
                }
                buttondata[i].count = 0;
            }
        }
        xSemaphoreGive(mButtonMutex);
        vTaskDelay(BUTTONTASK_UPDATE_TIME_MS/portTICK_PERIOD_MS);
    }
}

buttonstate_t getButtonPress(uint8_t button, bool reset) {
    buttonstate_t returnvalue = BUTTON_NOT_PRESSED;
    xSemaphoreTake(mButtonMutex, portMAX_DELAY);
    returnvalue = buttondata[button].state;
    if(reset) {
        buttondata[button].state = BUTTON_NOT_PRESSED;
    }
    xSemaphoreGive(mButtonMutex);
    return returnvalue;
}
void resetButtonState(void) {
    xSemaphoreTake(mButtonMutex, portMAX_DELAY);
    for(int i = 0; i < BUTTON_MAX; i++) {
        buttondata[i].state = BUTTON_NOT_PRESSED;
    }
    xSemaphoreGive(mButtonMutex);
}
void initButtonhandler(void) {
    mButtonMutex = xSemaphoreCreateMutex();
    xTaskCreate(buttonTask, "buttontask", 2*2048, NULL, 10, NULL);
}