#pragma once
#include <stdio.h>


int sdGetFileListLength(char* path);
int sdGetFileNameFromIndex(int index, char* path, char* namestorage);
bool sdCardPresent();
esp_err_t initSDReader(uint8_t mosi, uint8_t miso, uint8_t sck, uint8_t cs, int8_t sddetect);