#pragma once

#include "stdio.h"
/*--------------------------------------------------------------*/
/*                                                              */
/*-------------------------------------------------------------*/

typedef enum {
    AM_I2S_ES8388,
    AM_I2S_SGTL5000,
    AM_I2S_NAU88C22,
    AM_PWM,
} am_mode_t;

void am_init(am_mode_t mode, uint32_t samplerate, uint32_t buffersize, uint8_t sda, uint8_t scl);
void am_setVolumeMain(int volume);
void am_setVolumeOut1(int volume);
void am_setVolumeOut2(int volume);
int am_register_sender(uint32_t dataslots);
uint32_t am_getBufferSize(uint8_t sizeofElement);
uint32_t am_getBufferLevel(uint32_t senderID);
void am_send(void* data, uint32_t size, int senderID);
int am_receive(void* data);
void am_buffer_flush(uint32_t senderID);
