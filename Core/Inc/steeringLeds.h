#ifndef STEERING_LEDS_H
#define STEERING_LEDS_H

#include "main.h"
#include "stdbool.h"

extern uint8_t colors[][3];

extern bool ledDataSentFlag;
extern uint16_t rpm;
extern uint8_t rxBuf1[];
extern bool dataSent;

extern void ledInit();
extern void ledSetBrightness(const float brightness);
extern void ledSetColor(const uint8_t LEDnum, const uint8_t red,
                        const uint8_t green, const uint8_t blue);
extern void ledSend(const float brightness);
extern void ledUpdate();

#endif // STEERING_LEDS_H
