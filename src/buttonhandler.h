#pragma once

#define BUTTON_GPIO_REC     36
#define BUTTON_GPIO_MODE    39

#define BUTTON_REC          0
#define BUTTON_MODE         1

#define BUTTONTASK_UPDATE_TIME_MS      10
#define BUTTON_PRESS_SHORT_TIME_MS     100
#define BUTTON_PRESS_LONG_TIME_MS      1000

typedef enum {
    BUTTON_NOT_PRESSED,
    BUTTON_PRESSED_SHORT,
    BUTTON_PRESSED_LONG,
} buttonstate_t;

buttonstate_t getButtonPress(uint8_t button, bool reset);
void resetButtonState(void);
void initButtonhandler(void);