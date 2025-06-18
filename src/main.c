#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_pm.h"
#include "driver/gpio.h"

#include "tmf8820.h"
#include "audioplayer.h"
#include "buttonhandler.h"

#define TAG "lptof"

void app_main() {
    vTaskDelay(3000/portTICK_PERIOD_MS);
    // esp_pm_config_esp32s3_t config;
    // config.light_sleep_enable = true;
    // config.max_freq_mhz = 240;
    // config.min_freq_mhz = 20;
    // esp_pm_configure(&config);
    initButtonhandler();
    // initTMF8820(200, 25, 6, 100, 2000);
    initAudioPlayer();
    initSDCard();
    initEncoder();

    uint8_t volume = 20;
    setVolumeMain(volume);
    setVolumeOut1(volume);
    setVolumeOut2(volume);

    for(;;) {
        // ESP_LOGI(TAG, "Test");
        if(getButtonPress(BUTTON_REC, true) == BUTTON_PRESSED_SHORT) {
            ESP_LOGI(TAG, "Button Rec pressed short");
            initSDCard();
            
        }
        if(getButtonPress(BUTTON_MODE, true) == BUTTON_PRESSED_SHORT) {
            ESP_LOGI(TAG, "Button Mode pressed short");
            readFileListFromSD();
        }
        // if(tmf8820_detected()) {
        //     ESP_LOGI(TAG, "Found you! Distance: %i", tmf8820_distance());
        // }
        vTaskDelay(200/portTICK_PERIOD_MS);
    }
}