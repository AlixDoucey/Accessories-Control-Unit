#ifndef FILTER_H
#define FILTER_H

#include "main.h"
#include <stdint.h>

// Butterworth 2nd order IIR | cutoff 100hz | sampling rate 1khz
extern const float filter1[7];
// Butterworth 2nd order IIR | cutoff 100hz | sampling rate 333hz
extern const float filter2[7];

typedef float (*sample_converter)(const void *buffer, uint8_t index);

typedef enum {
  UINT16 = 0,
  INT32,
} buf_type;

typedef struct {
  uint8_t adc_buf_len;
  uint8_t adc_nbr_conversion;
  uint8_t rank; // rank of the adc sample
  void *adc_buf;
  buf_type type;
  sample_converter convert_sample;
  uint16_t *pending_samples; // counter incremented by the timer callBack
  uint8_t last_processed_idx;
  float buf[2];      // filter states variables
  const float *coef; // filter coefficients
} filter;

void filter_create(filter *inst, const float *preset, buf_type buf_type,
                   uint8_t adc_buf_len, uint8_t adc_nbr_conversion,
                   uint8_t rank, void *adc_buf, uint16_t *pending_samples);

float filter_process_sample(filter *inst);

#endif // FILTER_H
