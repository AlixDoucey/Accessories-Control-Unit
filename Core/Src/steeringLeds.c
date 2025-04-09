#include "math.h"
#include <stdint.h>

#include "app_threadx.h"
#include "ecu.h"
#include "gradient.h"
#include "software_timer.h"
#include "steeringLeds.h"

// clang-format off

uint8_t colors_base[11][3] = {
    {0, 255, 0},
    {0, 255, 0},
    {145, 223, 0},
    {145, 223, 0},
    {241, 80, 0},
    {241, 80, 0},
    {241, 80, 0},
    {255, 0, 0},
    {255, 0, 0},
    {0, 0, 255},
    {0, 0, 255},
};

// clang-format on

bool led_data_sent = false;

Led_data leds;
Led_data solid_black;
Led_data solid_red;

stim timer;

void led_init() {
  led_reset_colors(&leds);
  led_set_solid_color(0, 0, 0, 0, &solid_black);
  led_set_solid_color(255, 0, 0, BRIGHTNESS, &solid_red);

  stim_start(&timer);
}

void _led_set_all_brightness(const float brightness, Led_data *led) {
  float gamma = powf(brightness, GAMMA);

  if (gamma > 1)
    gamma = 1;
  else if (gamma < 0)
    gamma = 0;

  for (uint8_t i = 0; i < MAX_LED; i++) {
    uint16_t mask = 1 << i;
    if (led->gamma[i] != gamma) {
      led->gamma[i] = gamma;
      led->changelog |= mask;
    }
  }
}

void led_set_all_brightness(const float brightness) { _led_set_all_brightness(brightness, &leds); }

void _led_set_brightness(Led_data *led, uint8_t led_num, const float brightness) {
  float gamma = powf(brightness, GAMMA);

  if (gamma > 1)
    gamma = 1;
  else if (gamma < 0)
    gamma = 0;

  led->gamma[led_num] = gamma;
  led->changelog |= (1 << led_num);
}

void led_set_brightness(uint8_t led_num, const float brightness) { _led_set_brightness(&leds, led_num, brightness); }

void _led_compute_gamma(Led_data *led) {
  for (int i = 0; i < MAX_LED; i++) {
    uint16_t mask = 1 << i;

    if (led->changelog & mask) {
      for (int j = 0; j < 3; j++) {
        led->final_color[i][j] = (led->color[i][j]) * led->gamma[i];
      }
    }
  }
  led->changelog = 0x000;
}

// clang-format off

void _led_set_color(const uint8_t led_num,
                   const uint8_t red,
                   const uint8_t green,
                   const uint8_t blue,
                   const float brightness,
                   Led_data *led){
  float gamma = powf(brightness, GAMMA);

  if (gamma == 0){
    led->gamma[led_num] = gamma;
  } else {
    led->color[led_num][0] = red;
    led->color[led_num][1] = green;
    led->color[led_num][2] = blue;
    led->gamma[led_num] = gamma;
  }

  led->changelog = led->changelog | (1 << led_num);
}

void led_set_color(const uint8_t led_num,
                   const uint8_t red,
                   const uint8_t green,
                   const uint8_t blue,
                   const float brightness) {
  _led_set_color(led_num, red, green, blue, brightness,  &leds);
};

void led_set_solid_color(const uint8_t red,
                         const uint8_t green,
                         const uint8_t blue,
                         const float brightness,
                         Led_data *led){
  float gamma = powf(brightness, GAMMA);
  for (int i = 0; i < MAX_LED; i++) {
      led->color[i][0] = red;
      led->color[i][1] = green;
      led->color[i][2] = blue;
      led->gamma[i] = gamma;
  }
  led->changelog = 0x7FF;
}

// clang-format on
void led_reset_colors(Led_data *led) {
  for (int i = 0; i < MAX_LED; i++) {
    for (int j = 0; j < 3; j++) {
      led->color[i][j] = colors_base[i][j];
    }
  }
}

void _convert_to_pwm(Led_data *led, uint16_t *pwm_data) {
  uint32_t color, index = 0;

  for (int i = 0; i < MAX_LED; i++) {
    color = ((led->final_color[i][0] << 16) | (led->final_color[i][1] << 8) | (led->final_color[i][2]));
    for (int i = 23; i >= 0; i--) {
      if (color & (1 << i))
        pwm_data[index] = 212; // 312*0.68
      else
        pwm_data[index] = 100; // 312-212

      index++;
    }
  }
}

#define RESET_CODE 50
uint16_t pwm_data[(8 * 3 * MAX_LED) + RESET_CODE];

void _led_send(Led_data *led) {
  _led_compute_gamma(led);
  _convert_to_pwm(led, pwm_data);

  HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_3, (uint32_t *)pwm_data, sizeof(pwm_data));
  while (!led_data_sent)
    ;
  led_data_sent = false;
}

void led_send() { _led_send(&leds); }

void led_startup_animation() {
  for (int i = 0; i < 11; i++) {
    led_set_brightness(i, BRIGHTNESS);
    led_send();
    tx_thread_sleep(MS_TO_TICKS(100));
  }

  tx_thread_sleep(MS_TO_TICKS(500));

  for (float i = BRIGHTNESS; i >= 0; i -= 0.0025) {
    led_set_all_brightness(i);
    led_send();
    tx_thread_sleep(MS_TO_TICKS(5));
  }
}

#define MIN_RPM 4000
#define REDLINE 9000
#define MAX_RPM 9500

void _redline() {
#define MIN_BLINK_RATE 250 // in ms
#define MAX_BLINK_RATE 150 // in ms
  stim_start(&timer);
  // uint16_t blink_rate = roundf(MIN_BLINK_RATE + (((float)ecu.rpm - REDLINE) / (MAX_RPM - REDLINE)) * (MAX_BLINK_RATE
  // - MIN_BLINK_RATE));

  if (stim_get(&timer) % (MAX_BLINK_RATE * 2) < MAX_BLINK_RATE) {
    _led_send(&solid_red);
  } else {
    _led_send(&solid_black);
  }
}

void led_update() {
  static uint8_t num_on = 0;

  ecu_data_update();

  if (ecu.rpm > REDLINE) {
    _redline();
  } else {
    int16_t rpm_scaled = ecu.rpm - MIN_RPM;
    stim_stop(&timer);

    if (rpm_scaled < 0)
      rpm_scaled = 0;
    num_on = roundf((float)(rpm_scaled * MAX_LED) / (REDLINE - MIN_RPM));

    for (int i = 0; i < MAX_LED; i++) {
      float a;
      uint8_t idx;

      if (i <= num_on) {
        a = 1 - (float)rpm_scaled / (REDLINE - MIN_RPM);
        idx = roundf(0 + a * (100 - 0));
        if (i != num_on)
          led_set_color(i, gradient[idx][0], gradient[idx][1], gradient[idx][2], BRIGHTNESS);
        else {
          if (rpm_scaled != 0) {
            float rpm_scaled_prev_led = ((float)(num_on - 1) * (REDLINE - MIN_RPM) / (float)MAX_LED);
            float rpm_scaled_next_led = ((float)(num_on + 1) * (REDLINE - MIN_RPM) / (float)MAX_LED);
            float brightness = 0 + (rpm_scaled - rpm_scaled_prev_led) / (rpm_scaled_next_led - rpm_scaled_prev_led) *
                                       (float)(BRIGHTNESS - 0);

            led_set_color(i, gradient[idx][0], gradient[idx][1], gradient[idx][2], brightness);
          } else {

            led_set_color(i, 0, 0, 0, 0);
          }
        }
      } else {
        led_set_color(i, 0, 0, 0, 0);
      }
    }

    led_send();
  }
}
