#pragma once

// #include "freertos/FreeRTOS.h"
// #include "driver/i2s.h"
#include "driver/i2s_std.h"

#define SAMPLE_RATE     (44100)

#define I2S_NUM         (0)

#define I2S_BCK_IO      (GPIO_NUM_5)
#define I2S_WS_IO       (GPIO_NUM_25)
#define I2S_DO_IO       (GPIO_NUM_26)
#define I2S_DI_IO       (GPIO_NUM_35)
#define IS2_MCLK_PIN	(GPIO_NUM_0)

void i2smanager_init(uint32_t samplerate, i2s_data_bit_width_t bits_per_sample, uint8_t i2s_channel_nums);
esp_err_t i2smanager_read(void* data, size_t size, size_t *bytes_read, TickType_t ticks_to_wait);
esp_err_t i2smanager_write(uint8_t *data, size_t size, size_t *bytes_written, TickType_t ticks_to_wait);
esp_err_t i2smanager_zero_dma_buffer(void);