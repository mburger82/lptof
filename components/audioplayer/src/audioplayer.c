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

#include "sdreader.h"
#include "audiomanager.h"
#include "mad.h"

#include "../audioplayer.h"


#define TAG "AUDIOHANDLER"

#define GPIO_POWER_ENABLE_BITMASK  (1ULL<<GPIO_POWER_ENABLE)

typedef struct {
    char filepath[MAX_NAMELENGTH+10];
    FILE *fd;
    char fd_path[MAX_NAMELENGTH+10];
    uint8_t *dbuffer;
    int16_t *playbuffer;
    bool playing;
    bool stopping;
    uint32_t am_senderID;
}player_mp3_t;
player_mp3_t* player;

filedata_t filelist[MAX_FILES];

static enum mad_flow input(void *data, struct mad_stream *stream) {
    player_mp3_t *obj = data;
    if(obj->stopping) {
        obj->stopping = false;
        return MAD_FLOW_STOP; 
    }
    int keep = stream->bufend - stream->next_frame;
    fseek ( obj->fd , keep * -1 , SEEK_CUR );
    if(obj->dbuffer != NULL) {
        free(obj->dbuffer);
    }
    const size_t chunk_size = 2048;
    obj->dbuffer = malloc(chunk_size);
    if (NULL == obj->dbuffer) {
        printf("audio data dbuffer malloc failed");
        return MAD_FLOW_STOP;
    }
    int len = fread(obj->dbuffer, 1, chunk_size, obj->fd);
    if (len <= 0) {
        return MAD_FLOW_STOP;
    }
    mad_stream_buffer(stream, obj->dbuffer, len);
    if(len != chunk_size) {
        return MAD_FLOW_STOP;
    } else {
        return MAD_FLOW_CONTINUE;
    }    
}

static inline signed int scale(mad_fixed_t sample) {
  sample += (1L << (MAD_F_FRACBITS - 16));
  if (sample >= MAD_F_ONE)
    sample = MAD_F_ONE - 1;
  else if (sample < -MAD_F_ONE)
    sample = -MAD_F_ONE;
  return sample >> (MAD_F_FRACBITS + 1 - 16);
}
uint32_t current_average;
uint32_t get_current_average() {
    return current_average;
}

static enum mad_flow output(void *data, struct mad_header const *header, struct mad_pcm *pcm) {
    player_mp3_t *obj = data;
    if(obj->stopping) {
        obj->stopping = false;
        return MAD_FLOW_STOP; 
    }
    unsigned int nsamples;
    mad_fixed_t const *left_ch, *right_ch;
    nsamples = pcm->length;
    left_ch = pcm->samples[0];
    right_ch = pcm->samples[1];
    int i = 0;
    uint16_t mybuffer_audio[1152 * 4];
    while (nsamples--)
    {
        float volume = 0.5;
        mybuffer_audio[i++] = scale(*left_ch++)*volume;
        mybuffer_audio[i++] = scale(*right_ch++)*volume;
    }
    am_send(mybuffer_audio, 1152 * 4, obj->am_senderID);    
    return MAD_FLOW_CONTINUE;
}

static enum mad_flow error(void *data, struct mad_stream *stream, struct mad_frame *frame) {
  printf("decoding error 0x%04x (%s) \n", stream->error, mad_stream_errorstr(stream));
  /* return MAD_FLOW_BREAK here to stop decoding (and propagate an error) */
  return MAD_FLOW_CONTINUE;
}

void initPowerGPIO() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_POWER_ENABLE_BITMASK;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
}

esp_err_t initSDCard(void){
    return initSDReader(GPIO_SD_MOSI, GPIO_SD_MISO, GPIO_SD_SCK, GPIO_SD_CS, -1);
}

int readFileListFromSD(){
    int nfiles = sdGetFileListLength("");
    for(int i = 0; i < nfiles; i++) {
        sdGetFileNameFromIndex(i, "", filelist[i].name);
    }
    return nfiles;
}

void powerEnable(bool state) {
    if(state) {
        gpio_set_level(GPIO_POWER_ENABLE, 1);
    } else {
        gpio_set_level(GPIO_POWER_ENABLE, 0);
    }
}

void skip_id3v2_tag(FILE *fp) {
    unsigned char header[10];
    if (fread(header, 1, 10, fp) != 10) {
        fseek(fp, 0, SEEK_SET);
        return;
    }
    if (memcmp(header, "ID3", 3) == 0) {
        size_t tag_size =
            ((header[6] & 0x7F) << 21) |
            ((header[7] & 0x7F) << 14) |
            ((header[8] & 0x7F) << 7) |
            (header[9] & 0x7F);
        fseek(fp, 10 + tag_size, SEEK_SET);  // Skip tag
        // ESP_LOGI(TAG, "found ID3 Tag. skipping...");
    } else {
        fseek(fp, 0, SEEK_SET);  // Not ID3
        // ESP_LOGI(TAG, "found no ID3 Tag. not skipping...");
    }
}

EventGroupHandle_t ev_playercontrol;
#define CONTROL_START_MP3    1 << 0
#define CONTROL_STOP_MP3     1 << 1
#define CONTROL_IS_PLAYING   1 << 2
#define PLAYER_START         1 << 3
SemaphoreHandle_t mx_mp3name;
char mp3path[MAX_NAMELENGTH];

void audioControlTask(void* param) {
    ev_playercontrol = xEventGroupCreate();
    mx_mp3name = xSemaphoreCreateMutex();
    initPowerGPIO();
    for(;;) {
        EventBits_t playercontrol = xEventGroupGetBits(ev_playercontrol);
        if(playercontrol & CONTROL_START_MP3) {
            xEventGroupClearBits(ev_playercontrol, CONTROL_START_MP3);
            if(!(playercontrol & CONTROL_IS_PLAYING)) {
                xEventGroupSetBits(ev_playercontrol, PLAYER_START);
            } else {
                ESP_LOGI(TAG, "Player busy");
            }
        } else if(playercontrol & CONTROL_STOP_MP3) {
            xEventGroupClearBits(ev_playercontrol, CONTROL_STOP_MP3);
            if(player->playing == true) {
                player->stopping = true;
            }
        }
        vTaskDelay(20/portTICK_PERIOD_MS);
    }
}
void audioplayerTask(void* param){
    int mp3_senderID = am_register_sender(5);
    ESP_LOGI(TAG, "Created SenderID in audioplayertask: %i", (int)mp3_senderID);
    player = malloc(sizeof(player_mp3_t));        
    player->am_senderID = mp3_senderID;
    struct mad_decoder decoder;
    for(;;) {
        player->dbuffer = NULL;
        xEventGroupWaitBits(ev_playercontrol, PLAYER_START, true, false, portMAX_DELAY);
        ESP_LOGI(TAG, "Player start playing!");
        xEventGroupSetBits(ev_playercontrol, CONTROL_IS_PLAYING);
        player->stopping = false;
        player->playing = true;
        xSemaphoreTake(mx_mp3name, portMAX_DELAY);
        sprintf(player->filepath, "/sdcard/%s", mp3path);
        xSemaphoreGive(mx_mp3name);
        ESP_LOGI(TAG, "open file: %s", player->filepath);
        player->fd = fopen(player->filepath, "r");        
        if (player->fd == NULL) {
            printf("Failed to read existing file : %s \n", player->filepath);            
        } else {
            // ESP_LOGI(TAG, "skip id3 tag");
            skip_id3v2_tag(player->fd);
            // ESP_LOGI(TAG, "init decoder");
            mad_decoder_init(&decoder, player, input, 0, 0, output, error, 0);
            // ESP_LOGI(TAG, "run decoder");
            mad_decoder_run(&decoder, MAD_DECODER_MODE_SYNC);            
            // ESP_LOGI(TAG, "running");
            mad_decoder_finish(&decoder);
            player->playing = false;
            if(player->dbuffer != NULL) {
                free(player->dbuffer);
            }
            fclose(player->fd);

            am_buffer_flush(mp3_senderID);
            
            // ESP_LOGI(TAG, "MP3 finished playing");
            xEventGroupClearBits(ev_playercontrol, CONTROL_IS_PLAYING);
        }
        vTaskDelay(50/portTICK_PERIOD_MS);
    }
}
void playMP3(char* filepath) {
    xSemaphoreTake(mx_mp3name, portMAX_DELAY);
    sprintf(mp3path, "%s", filepath);
    xEventGroupSetBits(ev_playercontrol, CONTROL_START_MP3);
    xSemaphoreGive(mx_mp3name);
}
void stopMP3(void) {
    xEventGroupSetBits(ev_playercontrol, CONTROL_STOP_MP3);
}
bool isPlayingMP3() {
    if(xEventGroupGetBits(ev_playercontrol) & CONTROL_IS_PLAYING) {
        return true;
    }
    return false;
}
void setVolumeMain(int volume) {
    am_setVolumeMain(volume);
}
void setVolumeOut1(int volume) {
    am_setVolumeOut1(volume);
}
void setVolumeOut2(int volume) {
    am_setVolumeOut2(volume);
}

void initAudioPlayer() {
    am_init(AM_I2S_ES8388, 44100, 2048, GPIO_CODEC_I2C_SDA, GPIO_CODEC_I2C_SCL);
    xTaskCreate(audioControlTask, "audiocontroller", 2*2048, NULL, 5, NULL);
    xTaskCreate(audioplayerTask, "audioplayer", 10*2048, NULL, 5, NULL);
}