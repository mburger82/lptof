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

#include "audiohandler.h"


#define TAG "AUDIOHANDLER"
#define GPIO_AMP_ENABLE     21
#define GPIO_AMP_ENABLE_BITMASK  (1ULL<<GPIO_AMP_ENABLE)

StreamBufferHandle_t sb_rxdata;
typedef struct {
    char filepath[100];
    FILE *fd;
    char fd_path[100];
    uint8_t *dbuffer;
    int16_t *playbuffer;
    bool playing;
    bool stopping;
    uint32_t am_senderID;
}player_mp3_t;

SemaphoreHandle_t sem_endata;
uint32_t mp3_senderID = 0;

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
    // const size_t chunk_size = 4096;
    const size_t chunk_size = 4*4096;
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
    unsigned int nchannels, nsamples;
    mad_fixed_t const *left_ch, *right_ch;
    /* pcm->samplerate contains the sampling frequency */
    //printf("Samplerate: %d \n", pcm->samplerate);
    nchannels = pcm->channels;
    nsamples = pcm->length;
    left_ch = pcm->samples[0];
    right_ch = pcm->samples[1];
    // ESP_LOGI(TAG, "MP3 Decoded: channels: %i - length: %i", pcm->channels, pcm->length);
    int i = 0;
    uint16_t mybuffer_audio[1152 * 4];
    while (nsamples--)
    {
        // signed int sample;
        // // output sample(s) in 16-bit signed little-endian PCM
        float volume = 0.95;
        // sample = scale(*left_ch++);
        // mybuffer_audio[i++] = (float)(sample * volume);
        // sample = scale(*right_ch++);
        // mybuffer_audio[i++] = (float)(sample * volume);
        mybuffer_audio[i++] = scale(*left_ch++)*volume;
        // mybuffer_audio[i++] = scale(*right_ch++)*volume;
        mybuffer_audio[i++] = scale(*right_ch++)*0;
        // mybuffer_audio[i++] = scale(*left_ch++);
        // mybuffer_audio[i++] = scale(*right_ch++);
    }
    am_send(mybuffer_audio, 1152 * 4, obj->am_senderID);
    return MAD_FLOW_CONTINUE;
}

static enum mad_flow error(void *data, struct mad_stream *stream, struct mad_frame *frame) {
  printf("decoding error 0x%04x (%s) \n", stream->error, mad_stream_errorstr(stream));
  /* return MAD_FLOW_BREAK here to stop decoding (and propagate an error) */
  return MAD_FLOW_CONTINUE;
}

void audiohandlerTask(void* param) {
    vTaskDelay(100/portTICK_PERIOD_MS);
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_AMP_ENABLE_BITMASK;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(GPIO_AMP_ENABLE, 1);
    vTaskDelay(100/portTICK_PERIOD_MS);
    
    playMP3File("");
    for(;;) {
        // ESP_LOGI(TAG, "Running...");
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

void initSDCard(void){
    initSDReader(GPIO_SD_MOSI, GPIO_SD_MISO, GPIO_SD_SCK, GPIO_SD_CS, -1);
}
int readFileListFromSD(){
    return 0;
}

void initEncoder(void){
    am_init(AM_I2S_ES8388, 44100, 2048, GPIO_CODEC_I2C_SDA, GPIO_CODEC_I2C_SCL);
    sem_endata = xSemaphoreCreateMutex();
    sb_rxdata = xStreamBufferCreate(8*2048, 256);
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
        ESP_LOGI(TAG, "found ID3 Tag. skipping...");
    } else {
        fseek(fp, 0, SEEK_SET);  // Not ID3
        ESP_LOGI(TAG, "found no ID3 Tag. not skipping...");
    }
}

void playMP3File(char* filename){
    mp3_senderID = am_register_sender(5);
    ESP_LOGI(TAG, "Created SenderID on mp3test: %i", (int)mp3_senderID);
    player_mp3_t* player = malloc(sizeof(player_mp3_t));        
    player->am_senderID = mp3_senderID;
    struct mad_decoder decoder;
    int result;
    for(;;) {
        vTaskDelay(1000/portTICK_PERIOD_MS);
        player->dbuffer = NULL;
        // strcpy(player->filepath, "/sdcard/ack_40_whitcher_questsuccess.mp3\0");
        strcpy(player->filepath, "/sdcard/ack_10_success1.mp3\0");
        // strcpy(player->filepath, "/sdcard/fail_05_hellodarkness.mp3\0");
        player->fd = fopen(player->filepath, "r");        
        if (player->fd == NULL) {
            printf("Failed to read existing file : %s \n", player->filepath);            
        } else {
            skip_id3v2_tag(player->fd);
            vTaskDelay(10/portTICK_PERIOD_MS);
            mad_decoder_init(&decoder, player, input, 0, 0, output, error, 0);
            result = mad_decoder_run(&decoder, MAD_DECODER_MODE_SYNC);            
            mad_decoder_finish(&decoder);
            free(player->dbuffer);
            printf("Finished decoding \n");
            fclose(player->fd);
        }
        vTaskDelay(2000/portTICK_PERIOD_MS);
    }
}
void stopPlaying(void){

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

void initAudioHandler() {
    
    xTaskCreate(audiohandlerTask, "audiohandler", 10*2048, NULL, 5, NULL);
}