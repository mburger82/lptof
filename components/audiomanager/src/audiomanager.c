#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "freertos/message_buffer.h"
#include "esp_system.h"
// #include "esp_event.h"
// #include "esp_event_loop.h"
#include "esp_log.h"
#include "audiomanager.h"
#include "codec_es8388.h"

#define TAG "AUDIOMANAGER"

uint32_t audio_buffer_size = 2048;

EventGroupHandle_t ev_audiomanager_state;
#define AUDIOMANAGER_READY      1<<0

typedef struct {
    StreamBufferHandle_t sendbuffer;
    uint32_t slots;
    int senderID;
    void* next;
}am_senderdata_t;

am_senderdata_t* senderdata = NULL;

QueueHandle_t q_audio_receive;

void* getSender(int senderID) {
    am_senderdata_t* sender = senderdata;
    while(sender->senderID != senderID) {
        sender = sender->next;
        if(sender == NULL) {
            return NULL;
        }
    }
    return sender;
}
int addSender(uint32_t dataslots) {
    am_senderdata_t* sender = senderdata;
    am_senderdata_t* newSender = malloc(sizeof(am_senderdata_t));
    if(sender != NULL) {
        while(sender->next != NULL) {
            sender = sender->next;
        }
        sender->next = newSender;
    } else {
        sender = newSender;
        sender->senderID = 0;
        senderdata = sender;
    }
    newSender->next = NULL;
    newSender->slots = dataslots;
    newSender->senderID = sender->senderID+1;
    // newSender->sendqueue = xQueueCreate(dataslots, datalength);    
    newSender->sendbuffer = xStreamBufferCreate(audio_buffer_size * dataslots, audio_buffer_size);
    if(newSender->sendbuffer != NULL) {        
        return newSender->senderID;        
    } else {
        return -1;
    }
}

void audioTask(void* p) {
    ESP_LOGI(TAG, "Start audioTask in audiomanager");
    int16_t* buffer_read = malloc(audio_buffer_size);    
    int16_t* buffer_write = malloc(audio_buffer_size);
    size_t bytes_read = 0;
    size_t bytes_written = 0;
    int16_t* data = malloc(audio_buffer_size);
    am_senderdata_t* sender;
    xEventGroupSetBits(ev_audiomanager_state, AUDIOMANAGER_READY);
    for(;;) {
        
        es8388_read(buffer_read, audio_buffer_size, &bytes_read, 100);
        xQueueOverwrite(q_audio_receive, buffer_read);
        sender = senderdata;
        memset(buffer_write, 0, audio_buffer_size);
        while(sender != NULL) {
            if(xStreamBufferBytesAvailable(sender->sendbuffer) >= audio_buffer_size) {
                size_t size = xStreamBufferReceive(sender->sendbuffer, data, audio_buffer_size, 0);
                for(int i = 0; i < audio_buffer_size/sizeof(int16_t);i++) {
                    buffer_write[i]+= data[i];
                }
            }
            sender = sender->next;
        }
        es8388_write(buffer_write, audio_buffer_size, &bytes_written, 100);
    }
}

void am_init(am_mode_t mode, uint32_t samplerate, uint32_t buffersize, uint8_t sda, uint8_t scl) {
    ESP_LOGI(TAG, "Init audiomanager - samplerate: %i - buffersize: %i", (int)samplerate, (int)buffersize);
    ev_audiomanager_state = xEventGroupCreate();
    audio_buffer_size = buffersize;
    q_audio_receive = xQueueCreate(1, audio_buffer_size);
    switch(mode) {
        case AM_I2S_ES8388:
            es8388_init(samplerate, 16, 2, sda, scl);
        break;
        case AM_I2S_NAU88C22:

        break;
        case AM_I2S_SGTL5000:

        break;
        case AM_PWM:

        break;
    }
    xTaskCreate(audioTask, "audiomanager", 4096, NULL, 10, NULL);
    ESP_LOGI(TAG, "Init done");
}
void am_setVolume(int volume) {
    es8388_setVolume(volume);
}
int am_register_sender(uint32_t dataslots) {
    xEventGroupWaitBits(ev_audiomanager_state, AUDIOMANAGER_READY, false, false, portMAX_DELAY);    
    uint32_t senderID = addSender(dataslots);
    am_senderdata_t* sender = getSender(senderID);
    ESP_LOGW(TAG, "Audio-Sender %i registered: Buffersize: %i", (int)sender->senderID, (int)xMessageBufferSpacesAvailable(sender->sendbuffer));
    return senderID;
}
uint32_t am_getBufferSize(uint8_t sizeofElement) {
    xEventGroupWaitBits(ev_audiomanager_state, AUDIOMANAGER_READY, false, false, portMAX_DELAY);    
    if(sizeofElement == NULL) {
        return audio_buffer_size;
    } else {
        return audio_buffer_size/sizeofElement;
    }
}
uint32_t am_getBufferLevel(uint32_t senderID) {
    xEventGroupWaitBits(ev_audiomanager_state, AUDIOMANAGER_READY, false, false, portMAX_DELAY);    
    am_senderdata_t* sender = getSender(senderID);
    return xStreamBufferBytesAvailable(sender->sendbuffer);
}
void am_send(void* data, uint32_t size, int senderID) {
    xEventGroupWaitBits(ev_audiomanager_state, AUDIOMANAGER_READY, false, false, portMAX_DELAY);    
    am_senderdata_t* sender = getSender(senderID);
    xStreamBufferSend(sender->sendbuffer, data, size, portMAX_DELAY);
}
int am_receive(void* data) {
    xEventGroupWaitBits(ev_audiomanager_state, AUDIOMANAGER_READY, false, false, portMAX_DELAY);    
    return xQueueReceive(q_audio_receive, data, portMAX_DELAY);    
}