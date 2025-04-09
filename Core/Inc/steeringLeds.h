#ifndef STEERING_LEDS_H
#define STEERING_LEDS_H

#include "main.h"
#include "stdbool.h"
// clang-format off

extern bool led_data_sent;

#define MAX_LED 11
#define BRIGHTNESS 0.6f
#define GAMMA 4.0f

typedef struct {
  uint8_t color[MAX_LED][3];       // GRB
  float gamma[MAX_LED];            // Alpha
  uint8_t final_color[MAX_LED][3]; // GRB
  uint16_t changelog;              // BITFIELD
} Led_data;

extern void led_init();
extern void led_reset_colors(Led_data *led);
extern void led_set_all_brightness(const float brightness);
extern void led_set_color(const uint8_t led_num, 
                          const uint8_t red, 
                          const uint8_t green, 
                          const uint8_t blue,
                          const float brightness);

extern void led_set_solid_color(const uint8_t red, 
                                const uint8_t green, 
                                const uint8_t blue,
                                const float brightness,
                                Led_data *led);

extern void led_startup_animation();
extern void led_update();

#endif // STEERING_LEDS_H
