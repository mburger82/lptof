#pragma once

#include "audioplayerconfig.h"
#include "driver/i2s_std.h"

void i2smanager_init(uint32_t samplerate, i2s_data_bit_width_t bits_per_sample, uint8_t i2s_channel_nums);
esp_err_t i2smanager_read(void* data, size_t size, size_t *bytes_read, TickType_t ticks_to_wait);
esp_err_t i2smanager_write(uint8_t *data, size_t size, size_t *bytes_written, TickType_t ticks_to_wait);