/*
 This is an STM32 library written for the NAU7802 24-bit wheatstone
 bridge and load cell amplifier, using the HAL library.
 Adapted from the Arduino library by Nathan Seidle @ SparkFun Electronics

 Modified and ported to STM32 using HAL by Yasir Shahzad
 Copyright (c) 2023 Yasir Shahzad (yasirshahzad918@gmail.com)

 Original Arduino library:
 https://github.com/sparkfun/SparkFun_NAU7802_Scale_Arduino_Library
 */

#include "NAU7802.h"

I2C_HandleTypeDef *NAU_hi2c;            // I2C handle
const uint8_t NAU_deviceAddress = 0x2A; // Default unshifted 7-bit address of the NAU7802

// y = mx+b
int32_t NAU_zeroOffset = 0;        // This is b
float NAU_calibrationFactor = 1.0; // This is m. User provides this number so that we can output y when requested
unsigned long NAU_ldoRampDelay =
    250; // During begin, wait this many millis after configuring the LDO before performing calibrateAFE
bool Initialized = false;

// Sets up the NAU7802 for basic function
// If initialize is true (or not specified), default init and calibration is performed
// If initialize is false, then it's up to the caller to initalize and calibrate
// Returns true upon completion
bool NAU_begin(I2C_HandleTypeDef *hi2c, bool initialize) {
  // Get user's options
  NAU_hi2c = hi2c;

  // Check if the device ack's over I2C
  if (NAU_isConnected() == false) {
    // There are rare times when the sensor is occupied and doesn't ack. A 2nd try resolves this.
    if (NAU_isConnected() == false)
      return (false);
  }

  bool result = true; // Accumulate a result as we do the setup

  if (initialize) {
    result &= NAU_reset(); // Reset all registers

    result &= NAU_powerUp(); // Power on analog and digital sections of the scale

    result &= NAU_setLDO(NAU7802_LDO_3V0); // Set LDO to 3.3V

    result &= NAU_setGain(NAU7802_GAIN_128); // Set gain to 128

    result &= NAU_setSampleRate(NAU7802_SPS_320); // Set samples per second to 320

    // Turn off CLK_CHP. From 9.1 power on sequencing.
    uint8_t adc = NAU_getRegister(NAU7802_ADC);
    adc |= 0x30;
    result &= NAU_setRegister(NAU7802_ADC, adc);

    result &= NAU_setBit(NAU7802_PGA_PWR_PGA_CAP_EN,
                         NAU7802_PGA_PWR); // Enable 330pF decoupling cap on chan 2. From 9.14 application circuit note.

    result &= NAU_clearBit(
        NAU7802_PGA_LDOMODE,
        NAU7802_PGA); // Ensure LDOMODE bit is clear - improved accuracy and higher DC gain, with ESR < 1 ohm

    tx_thread_sleep(NAU_ldoRampDelay); // Wait for LDO to stabilize - takes about 200ms

    NAU_getWeight(true, 10, 1000); // Flush

    result &= NAU_calibrateAFE(
        NAU7802_CALMOD_INTERNAL); // Re-cal analog front end when we change gain, sample rate, or channel

    Initialized = true;
  }

  return (result);
}

// Returns true if device is present
// Tests for device ack to I2C address
bool NAU_isConnected() {
  if (HAL_I2C_IsDeviceReady(NAU_hi2c, NAU_deviceAddress << 1, 3, 100) != HAL_OK)
    return (false); // Sensor did not ACK
  return (true);    // All good
}

// Returns true if Cycle Ready bit is set (conversion is complete)
bool NAU_available() { return (NAU_getBit(NAU7802_PU_CTRL_CR, NAU7802_PU_CTRL)); }

// Calibrate analog front end of system. Returns true if CAL_ERR bit is 0 (no error)
// Takes approximately 344ms to calibrate; wait up to 1000ms.
// It is recommended that the AFE be re-calibrated any time the gain, SPS, or channel number is changed.
bool NAU_calibrateAFE(NAU7802_Cal_Mode mode) {
  NAU_beginCalibrateAFE(mode);
  return NAU_waitForCalibrateAFE(1000);
}

// Begin asynchronous calibration of the analog front end.
//  Poll for completion with calAFEStatus() or wait with waitForCalibrateAFE()
void NAU_beginCalibrateAFE(NAU7802_Cal_Mode mode) {
  uint8_t value = NAU_getRegister(NAU7802_CTRL2);
  value &= 0xFC; // Clear CALMOD bits
  uint8_t calMode = (uint8_t)mode;
  calMode &= 0x03;  // Limit mode to 2 bits
  value |= calMode; // Set the mode
  NAU_setRegister(NAU7802_CTRL2, value);

  NAU_setBit(NAU7802_CTRL2_CALS, NAU7802_CTRL2);
}

// Check calibration status.
NAU7802_Cal_Status NAU_calAFEStatus() {
  if (NAU_getBit(NAU7802_CTRL2_CALS, NAU7802_CTRL2)) {
    return NAU7802_CAL_IN_PROGRESS;
  }

  if (NAU_getBit(NAU7802_CTRL2_CAL_ERROR, NAU7802_CTRL2)) {
    return NAU7802_CAL_FAILURE;
  }

  // Calibration passed
  return NAU7802_CAL_SUCCESS;
}

// Wait for asynchronous AFE calibration to complete with optional timeout.
// If timeout is not specified (or set to 0), then wait indefinitely.
// Returns true if calibration completes succsfully, otherwise returns false.
bool NAU_waitForCalibrateAFE(unsigned long timeout_ms) {
  unsigned long startTime = HAL_GetTick();
  NAU7802_Cal_Status cal_ready;

  while ((cal_ready = NAU_calAFEStatus()) == NAU7802_CAL_IN_PROGRESS) {
    if ((timeout_ms > 0) && ((HAL_GetTick() - startTime) > timeout_ms)) {
      break;
    }
    HAL_Delay(1);
  }

  if (cal_ready == NAU7802_CAL_SUCCESS) {
    return (true);
  }
  return (false);
}

// Set the readings per second
// 10, 20, 40, 80, and 320 samples per second is available
bool NAU_setSampleRate(uint8_t rate) {
  if (rate > 0b111)
    rate = 0b111; // Error check

  uint8_t value = NAU_getRegister(NAU7802_CTRL2);
  value &= 0b10001111; // Clear CRS bits
  value |= rate << 4;  // Mask in new CRS bits

  return (NAU_setRegister(NAU7802_CTRL2, value));
}

// Select between 1 and 2
bool NAU_setChannel(uint8_t channelNumber) {
  if (channelNumber == NAU7802_CHANNEL_1)
    return (NAU_clearBit(NAU7802_CTRL2_CHS, NAU7802_CTRL2)); // Channel 1 (default)
  else
    return (NAU_setBit(NAU7802_CTRL2_CHS, NAU7802_CTRL2)); // Channel 2
}

// Power up digital and analog sections of scale
bool NAU_powerUp() {
  NAU_setBit(NAU7802_PU_CTRL_PUD, NAU7802_PU_CTRL);
  NAU_setBit(NAU7802_PU_CTRL_PUA, NAU7802_PU_CTRL);

  // Wait for Power Up bit to be set - takes approximately 200us
  uint8_t counter = 0;
  while (1) {
    if (NAU_getBit(NAU7802_PU_CTRL_PUR, NAU7802_PU_CTRL) == true)
      break; // Good to go
    HAL_Delay(1);
    if (counter++ > 100)
      return (false); // Error
  }
  return (NAU_setBit(NAU7802_PU_CTRL_CS, NAU7802_PU_CTRL)); // Set Cycle Start bit. See 9.1 point 5
}

// Puts scale into low-power mode
bool NAU_powerDown() {
  NAU_clearBit(NAU7802_PU_CTRL_PUD, NAU7802_PU_CTRL);
  return (NAU_clearBit(NAU7802_PU_CTRL_PUA, NAU7802_PU_CTRL));
}

// Resets all registers to Power Of Defaults
bool NAU_reset() {
  NAU_setBit(NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL); // Set RR
  HAL_Delay(1);
  return (NAU_clearBit(NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL)); // Clear RR to leave reset state
}

// Set the onboard Low-Drop-Out voltage regulator to a given value
// 2.4, 2.7, 3.0, 3.3, 3.6, 3.9, 4.2, 4.5V are available
bool NAU_setLDO(uint8_t ldoValue) {
  if (ldoValue > 0b111)
    ldoValue = 0b111; // Error check

  // Set the value of the LDO
  uint8_t value = NAU_getRegister(NAU7802_CTRL1);
  value &= 0b11000111;    // Clear LDO bits
  value |= ldoValue << 3; // Mask in new LDO bits
  NAU_setRegister(NAU7802_CTRL1, value);

  return (NAU_setBit(NAU7802_PU_CTRL_AVDDS, NAU7802_PU_CTRL)); // Enable the internal LDO
}

void NAU_setLDORampDelay(unsigned long delay) { NAU_ldoRampDelay = delay; }
unsigned long NAU_getLDORampDelay() { return NAU_ldoRampDelay; }

// Set the gain
// x1, 2, 4, 8, 16, 32, 64, 128 are avaialable
bool NAU_setGain(uint8_t gainValue) {
  if (gainValue > 0b111)
    gainValue = 0b111; // Error check

  uint8_t value = NAU_getRegister(NAU7802_CTRL1);
  value &= 0b11111000; // Clear gain bits
  value |= gainValue;  // Mask in new bits

  return (NAU_setRegister(NAU7802_CTRL1, value));
}

// Get the revision code of this IC
uint8_t NAU_getRevisionCode() {
  uint8_t revisionCode = NAU_getRegister(NAU7802_DEVICE_REV);
  return (revisionCode & 0x0F);
}

// Returns 24-bit reading
// Assumes CR Cycle Ready bit (ADC conversion complete) has been checked to be 1
int32_t NAU_getReading() {
  if (Initialized)
    return NAU_get24BitRegister(NAU7802_ADCO_B2);
  return -1;
}

// Return the average of a given number of readings
// Gives up after 1000ms so don't call this function to average 8 samples setup at 1Hz output (requires 8s)
int32_t NAU_getAverage(uint8_t averageAmount, unsigned long timeout_ms) {
  int32_t total = 0; // Readings are 24-bit. We're good to average 255 if needed
  uint8_t samplesAquired = 0;

  unsigned long startTime = HAL_GetTick();
  while (1) {
    if (NAU_available() == true) {
      total += NAU_getReading();
      if (++samplesAquired == averageAmount)
        break; // All done
    }
    if (HAL_GetTick() - startTime > timeout_ms)
      return (0); // Timeout - Bail with error
    HAL_Delay(1);
  }
  total /= averageAmount;

  return (total);
}

// Call when scale is setup, level, at running temperature, with nothing on it
void NAU_calculateZeroOffset(uint8_t averageAmount, unsigned long timeout_ms) {
  NAU_setZeroOffset(NAU_getAverage(averageAmount, timeout_ms));
}

// Sets the internal variable. Useful for users who are loading values from NVM.
void NAU_setZeroOffset(int32_t newZeroOffset) { NAU_zeroOffset = newZeroOffset; }

int32_t NAU_getZeroOffset() { return (NAU_zeroOffset); }

// Call after zeroing. Provide the float weight sitting on scale. Units do not matter.
void NAU_calculateCalibrationFactor(float weightOnScale, uint8_t averageAmount, unsigned long timeout_ms) {
  int32_t onScale = NAU_getAverage(averageAmount, timeout_ms);
  float newCalFactor = ((float)(onScale - NAU_zeroOffset)) / weightOnScale;
  NAU_setCalibrationFactor(newCalFactor);
}

// Pass a known calibration factor into library. Helpful if users is loading settings from NVM.
// If you don't know your cal factor, call setZeroOffset(), then calculateCalibrationFactor() with a known weight
void NAU_setCalibrationFactor(float newCalFactor) { NAU_calibrationFactor = newCalFactor; }

float NAU_getCalibrationFactor() { return (NAU_calibrationFactor); }

// Returns the y of y = mx + b using the current weight on scale, the cal factor, and the offset.
float NAU_getWeight(bool allowNegativeWeights, uint8_t samplesToTake, unsigned long timeout_ms) {
  int32_t onScale = NAU_getAverage(samplesToTake, timeout_ms);

  // Prevent the current reading from being less than zero offset
  // This happens when the scale is zero'd, unloaded, and the load cell reports a value slightly less than zero value
  // causing the weight to be negative or jump to millions of pounds
  if (allowNegativeWeights == false) {
    if (onScale < NAU_zeroOffset)
      onScale = NAU_zeroOffset; // Force reading to zero
  }

  float weight = ((float)(onScale - NAU_zeroOffset)) / NAU_calibrationFactor;
  return (weight);
}

// Set Int pin to be high when data is ready (default)
bool NAU_setIntPolarityHigh() {
  return (NAU_clearBit(NAU7802_CTRL1_CRP, NAU7802_CTRL1)); // 0 = CRDY pin is high active (ready when 1)
}

// Set Int pin to be low when data is ready
bool NAU_setIntPolarityLow() {
  return (NAU_setBit(NAU7802_CTRL1_CRP, NAU7802_CTRL1)); // 1 = CRDY pin is low active (ready when 0)
}

// Mask & set a given bit within a register
bool NAU_setBit(uint8_t bitNumber, uint8_t registerAddress) {
  uint8_t value = NAU_getRegister(registerAddress);
  value |= (1 << bitNumber); // Set this bit
  return (NAU_setRegister(registerAddress, value));
}

// Mask & clear a given bit within a register
bool NAU_clearBit(uint8_t bitNumber, uint8_t registerAddress) {
  uint8_t value = NAU_getRegister(registerAddress);
  value &= ~(1 << bitNumber); // Set this bit
  return (NAU_setRegister(registerAddress, value));
}

// Return a given bit within a register
bool NAU_getBit(uint8_t bitNumber, uint8_t registerAddress) {
  uint8_t value = NAU_getRegister(registerAddress);
  value &= (1 << bitNumber); // Clear all but this bit
  return (value);
}

// Get contents of a register
uint8_t NAU_getRegister(uint8_t registerAddress) {
  uint8_t value;
  if (HAL_I2C_Mem_Read(NAU_hi2c, NAU_deviceAddress << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, &value, 1, 1000) !=
      HAL_OK)
    return -1; // Error
  return value;
}

// Send a given value to be written to given address
// Return true if successful
bool NAU_setRegister(uint8_t registerAddress, uint8_t value) {
  if (HAL_I2C_Mem_Write(NAU_hi2c, NAU_deviceAddress << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, &value, 1, 1000) !=
      HAL_OK)
    return false; // Error
  return true;
}

// Get contents of a 24-bit signed register (conversion result and offsets)
int32_t NAU_get24BitRegister(uint8_t registerAddress) {
  uint8_t data[3];
  if (HAL_I2C_Mem_Read(NAU_hi2c, NAU_deviceAddress << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, data, 3, 1000) !=
      HAL_OK)
    return 0; // Error

  union {
    uint32_t unsigned32;
    int32_t signed32;
  } signedUnsigned32; // Avoid ambiguity

  signedUnsigned32.unsigned32 = (uint32_t)data[0] << 16; // MSB
  signedUnsigned32.unsigned32 |= (uint32_t)data[1] << 8; // MidSB
  signedUnsigned32.unsigned32 |= (uint32_t)data[2];      // LSB

  if ((signedUnsigned32.unsigned32 & 0x00800000) == 0x00800000)
    signedUnsigned32.unsigned32 |= 0xFF000000; // Preserve 2's complement

  return signedUnsigned32.signed32;
}

// Send 24 LSBs of value to given register address. Return true if successful
// Note: assumes bit 23 is already correct for 24-bit signed
bool NAU_set24BitRegister(uint8_t registerAddress, int32_t value) {
  uint8_t data[3];
  union {
    uint32_t unsigned32;
    int32_t signed32;
  } signedUnsigned32; // Avoid ambiguity

  signedUnsigned32.signed32 = value;

  data[0] = (signedUnsigned32.unsigned32 >> 16) & 0xFF; // MSB
  data[1] = (signedUnsigned32.unsigned32 >> 8) & 0xFF;  // MidSB
  data[2] = signedUnsigned32.unsigned32 & 0xFF;         // LSB

  if (HAL_I2C_Mem_Write(NAU_hi2c, NAU_deviceAddress << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, data, 3, 1000) !=
      HAL_OK)
    return false; // Error

  return true;
}

// Get contents of a 32-bit register (gains)
uint32_t NAU_get32BitRegister(uint8_t registerAddress) {
  uint8_t data[4];
  if (HAL_I2C_Mem_Read(NAU_hi2c, NAU_deviceAddress << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, data, 4, 1000) !=
      HAL_OK)
    return 0; // Error

  uint32_t value = (uint32_t)data[0] << 24; // MSB
  value |= (uint32_t)data[1] << 16;
  value |= (uint32_t)data[2] << 8;
  value |= (uint32_t)data[3]; // LSB

  return value;
}

// Send 32-bit value to given register address. Return true if successful
bool NAU_set32BitRegister(uint8_t registerAddress, uint32_t value) {
  uint8_t data[4];
  data[0] = (value >> 24) & 0xFF; // MSB
  data[1] = (value >> 16) & 0xFF;
  data[2] = (value >> 8) & 0xFF;
  data[3] = value & 0xFF; // LSB

  if (HAL_I2C_Mem_Write(NAU_hi2c, NAU_deviceAddress << 1, registerAddress, I2C_MEMADD_SIZE_8BIT, data, 4, 1000) !=
      HAL_OK)
    return false; // Error

  return true;
}

// Helper methods
int32_t NAU_getChannel1Offset() { return NAU_get24BitRegister(NAU7802_OCAL1_B2); }
bool NAU_setChannel1Offset(int32_t value) { return NAU_set24BitRegister(NAU7802_OCAL1_B2, value); }
uint32_t NAU_getChannel1Gain() { return NAU_get32BitRegister(NAU7802_GCAL1_B3); }
bool NAU_setChannel1Gain(uint32_t value) { return NAU_set32BitRegister(NAU7802_GCAL1_B3, value); }
