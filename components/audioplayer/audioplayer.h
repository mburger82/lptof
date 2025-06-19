#pragma once

#include "audioplayerconfig.h"

typedef struct {
    char name[MAX_NAMELENGTH];
} filedata_t;
extern filedata_t filelist[MAX_FILES];

esp_err_t initSDCard(void);
int readFileListFromSD();

void powerEnable(bool state);

void playMP3(char* filepath);
void stopMP3(void);
bool isPlayingMP3();
void setVolume(int volume);
void setVolumeMain(int volume);
void setVolumeOut1(int volume);
void setVolumeOut2(int volume);

void initAudioPlayer();