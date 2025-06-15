#pragma once
#include <stdio.h>


bool sdCardPresetn();
void initSDReader(uint8_t mosi, uint8_t miso, uint8_t sck, uint8_t cs, int8_t sddetect);