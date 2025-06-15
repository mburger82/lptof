#pragma once

#include "stdio.h"
// I2S Interface
#define I2S_NUM I2S_NUM_0

/*--------------------------------------------------------------*/
/*                                                              */
/*-------------------------------------------------------------*/

// #define AUDIO_BUFF_LEN 2048

typedef enum {
    AM_I2S_ES8388,
    AM_I2S_SGTL5000,
    AM_I2S_NAU88C22,
    AM_PWM,
} am_mode_t;

void am_init(am_mode_t mode, uint32_t samplerate, uint32_t buffersize, uint8_t sda, uint8_t scl);
int am_register_sender(uint32_t dataslots);
uint32_t am_getBufferSize(uint8_t sizeofElement);
uint32_t am_getBufferLevel(uint32_t senderID);
void am_send(void* data, uint32_t size, int senderID);
int am_receive(void* data);
