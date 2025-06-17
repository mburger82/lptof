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
#include "audiohandler.h"
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
    initAudioHandler();
    initSDCard();
    initEncoder();

    uint8_t volume = 100;
    setVolume(volume);

    for(;;) {
        // ESP_LOGI(TAG, "Test");
        if(getButtonPress(BUTTON_REC, true) == BUTTON_PRESSED_SHORT) {
            ESP_LOGI(TAG, "Button Rec pressed short");
            if(volume < 100) {
                volume += 10;
            }
            setVolume(volume);
            // playMP3File("");
        }
        if(getButtonPress(BUTTON_MODE, true) == BUTTON_PRESSED_SHORT) {
            ESP_LOGI(TAG, "Button Mode pressed short");
            if(volume > 0) {
                volume -= 10;
            }
            setVolume(volume);
        }
        // if(tmf8820_detected()) {
        //     ESP_LOGI(TAG, "Found you! Distance: %i", tmf8820_distance());
        // }
        vTaskDelay(200/portTICK_PERIOD_MS);
    }
}