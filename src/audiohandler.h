#pragma once

#define GPIO_CODEC_I2C_SDA          33
#define GPIO_CODEC_I2C_SCL          32

#define GPIO_I2S_MCLK               0
#define GPIO_I2S_BCLK               27
#define GPIO_I2S_LRCLK              25
#define GPIO_I2S_DIN                -1
#define GPIO_I2S_DOUT               26

#define GPIO_SD_MOSI                15
#define GPIO_SD_MISO                2
#define GPIO_SD_SCK                 14
#define GPIO_SD_CS                  13
#define GPIO_SD_CD                  34

#define MAX_FILES                   50

typedef struct {
    char name[50];
} filedata_t;
extern filedata_t filelist[MAX_FILES];

void initSDCard(void);
int readFileListFromSD();

void initEncoder();
void playMP3File(char* filename);
void stopPlaying();


void initAudiomanager();