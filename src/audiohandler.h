#pragma once

#define GPIO_CODEC_I2C_SDA          18
#define GPIO_CODEC_I2C_SCL          23

#define GPIO_I2S_MCLK               0
#define GPIO_I2S_BCLK               5
#define GPIO_I2S_LRCLK              25
#define GPIO_I2S_DIN                26
#define GPIO_I2S_DOUT               35

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
void setVolume(int volume);
void setVolumeMain(int volume);
void setVolumeOut1(int volume);
void setVolumeOut2(int volume);

void initAudioHandler();