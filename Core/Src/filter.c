#include "filter.h"
#include "main.h"
#include <stdint.h>

#define b0 0
#define b1 1
#define b2 2
#define a0 3
#define a1 4
#define a2 5
#define gain 6

// Butterworth 2nd order IIR | cutoff 100hz | sampling rate 1khz
const float filter1[7] = {1,
                          2,
                          1,
                          1,
                          -1.14298045635223388671875,
                          0.4128015935420989990234375,
                          0.06745527684688568115234375};

// Butterworth 2nd order IIR | cutoff 100hz | sampling rate 333hz
const float filter2[7] = {1,
                          2,
                          1,
                          1,
                          0.3717645108699798583984375,
                          0.19611142575740814208984375,
                          0.3919689953327178955078125};

float convert_uint16(const void *buffer, uint8_t index) {
  const uint16_t *buf = (const uint16_t *)buffer;
  return (float)buf[index];
}

float convert_int32(const void *buffer, uint8_t index) {
  const int32_t *buf = (const int32_t *)buffer;
  return (float)buf[index];
}

void filter_create(filter *inst, const float *preset, buf_type buf_type, uint8_t adc_buf_len, uint8_t adc_nbr_conversion,
                   uint8_t rank, void *adc_buf, uint16_t *pending_samples) {
  inst->adc_buf_len = adc_buf_len;
  inst->adc_nbr_conversion = adc_nbr_conversion;
  inst->rank = rank;
  inst->adc_buf = adc_buf;
  inst->pending_samples = pending_samples;
  inst->last_processed_idx = rank - 1 + adc_buf_len - adc_nbr_conversion;
  inst->coef = preset;

  switch (buf_type) {
  case UINT16:
    inst->convert_sample = convert_uint16;
    break;
  case INT32:
    inst->convert_sample = convert_int32;
    break;
  }
}

float _process_sample(filter *inst, uint8_t latest_sample_idx) {
  // Convert sample
  float latest_sample = inst->convert_sample(inst->adc_buf, latest_sample_idx);

  // Calculate new state
  float new_state = latest_sample - inst->coef[a1] * inst->buf[0] - inst->coef[a2] * inst->buf[1];

  float output =
      inst->coef[gain] * (inst->coef[b0] * new_state + inst->coef[b1] * inst->buf[0] + inst->coef[b2] * inst->buf[1]);

  // Update states
  inst->buf[1] = inst->buf[0];
  inst->buf[0] = new_state;

  return output;
}

uint8_t _increment_idx(filter *inst, uint8_t sample_idx) {
  sample_idx = inst->last_processed_idx + inst->adc_nbr_conversion;
  if (sample_idx >= inst->adc_buf_len) {
    sample_idx = sample_idx - inst->adc_buf_len;
  }

  return sample_idx;
}

float filter_process_sample(filter *inst) {
  float output_sample = 0;
  uint16_t _pending_samples = *inst->pending_samples;
  *inst->pending_samples = 0;
  uint8_t sample_idx = 0;

  if (_pending_samples != 0) {
    if (_pending_samples > inst->adc_buf_len / inst->adc_nbr_conversion)
      _pending_samples %= inst->adc_buf_len / inst->adc_nbr_conversion;
    for (uint8_t i = 0; i < _pending_samples; i++) {
      sample_idx = _increment_idx(inst, sample_idx);
      output_sample = roundf(_process_sample(inst, sample_idx));
      inst->last_processed_idx = sample_idx;
    }
  }
  return output_sample;
}
