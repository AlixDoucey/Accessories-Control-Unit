#include "steeringLeds.h"
#include "math.h"

uint8_t colors[11][3] = {
    {255, 0, 0},   // LED 0
    {255, 0, 0},   // LED 1
    {223, 145, 0}, // LED 2
    {223, 145, 0}, // LED 3
    {186, 196, 0}, // LED 4
    {255, 255, 0}, // LED 5
    {167, 215, 0}, // LED 6
    {80, 241, 0},  // LED 7
    {80, 241, 0},  // LED 8
    {0, 255, 0},   // LED 9
    {0, 255, 0}    // LED 10
};

bool ledDataSentFlag = false;
uint16_t rpm = 0;
uint8_t rxBuf1[80];
bool dataSent = true;

// Storing the LED data
#define MAX_LED 11
uint8_t LED_Data[MAX_LED][4]; // before brightness correction
uint8_t LED_Mod[MAX_LED][4];  // after brightness correction

void ledInit() {
  for (int i = 0; i < 11; ++i) {
    for (int j = 0; j < 4; ++j) {
      LED_Data[i][j] = 0;
      LED_Mod[i][j] = 0;
    }
  }
}

void ledSetBrightness(const float brightness) {
  float gammaBrightness = powf(brightness, 2.2);

  if (gammaBrightness > 1)
    gammaBrightness = 1;
  else if (gammaBrightness < 0)
    gammaBrightness = 0;

  for (int i = 0; i < MAX_LED; i++) {
    LED_Mod[i][0] = LED_Data[i][0];
    for (int j = 1; j < 4; j++) {
      LED_Mod[i][j] = (LED_Data[i][j]) * gammaBrightness;
    }
  }
}

void ledSetColor(const uint8_t LEDnum, const uint8_t red, const uint8_t green,
                 const uint8_t blue) {
  LED_Data[LEDnum][0] = LEDnum;
  LED_Data[LEDnum][1] = green;
  LED_Data[LEDnum][2] = red;
  LED_Data[LEDnum][3] = blue;
}

// Convert and send the data to DMA
uint16_t pwmData[(24 * MAX_LED) + 50];

void ledSend(const float brightness) {
  ledSetBrightness(brightness);

  uint32_t color, index = 0;

  for (int i = 0; i < MAX_LED; i++) {
    color = ((LED_Mod[i][1] << 16) | (LED_Mod[i][2] << 8) | (LED_Mod[i][3]));
    for (int i = 23; i >= 0; i--) {
      if (color & (1 << i)) {
        pwmData[index] = 76; // 112*0.68
      }

      else
        pwmData[index] = 36; // 112-76

      index++;
    }
  }

  for (int i = 0; i < 50; i++) {
    pwmData[index] = 0;
    index++;
  }

  HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3, (uint32_t *)pwmData, index);
  while (!ledDataSentFlag)
    ;
  ledDataSentFlag = false;
}

// Update Leds (Rpm calculations)
void ledUpdate() {
#define MAX_RPM 4300 // (9300-5000)

  uint16_t num_on = roundf(((rpm - 4300.0) * MAX_LED) /
                           MAX_RPM); // RPM-5000 because MAX_RPM=10000-5000

  if (num_on < 0 || num_on > 11) {
    num_on = 0;
  }
  // Loop through the LEDs from 0 to 10
  for (int i = 0; i < 11; i++) {
    // If the LED index is less than the input, turn it on with the predefined
    // color
    if (i < num_on) {
      ledSetColor(i, colors[i][0], colors[i][1], colors[i][2]);
    }
    // Otherwise, turn it off by setting the color to 0
    else {
      ledSetColor(i, 0, 0, 0);
    }
  }
}
