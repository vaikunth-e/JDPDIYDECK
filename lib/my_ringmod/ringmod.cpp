#include "ringmod.h"

// Q15 fixed-point macro
// 1.0 becomes 32768 (1 * 2^15)
#define Q15(x) ((int32_t)((x) * 32768.0f))

namespace {

// Default ring modulation settings
static float carrierHz = 120.0f;

// Q15 dry/wet gains
// 32768 = 1.0
static int32_t dryQ15 = Q15(0.70f);
static int32_t wetQ15 = Q15(0.40f);

// 32-bit phase accumulator
static uint32_t phase = 0;
static uint32_t phaseInc = 0;

static inline int32_t clampQ15Gain(float value) {
  if (value < 0.0f) value = 0.0f;
  if (value > 1.0f) value = 1.0f;
  return Q15(value);
}

static inline int16_t triangleCarrierQ15(uint32_t phaseValue) {
  // Use the top 16 bits of the 32-bit phase accumulator
  uint32_t p = phaseValue >> 16;  // 0 to 65535

  // Convert saw phase into triangle shape
  // tri goes roughly 0 -> 65534 -> 0
  uint32_t tri;

  if (p < 32768) {
    tri = p * 2;
  } else {
    tri = (65535 - p) * 2;
  }

  // Convert 0 to 65534 to approximately -32767 to +32767.
  return (int16_t)((int32_t)tri - 32767);
}

} // anonymous namespace

void ringModInit(uint32_t sampleRateHz) {
  phase = 0;
  ringModSetCarrierHz(carrierHz, sampleRateHz);
  ringModSetMix(0.70f, 0.40f);
}

void ringModSetCarrierHz(float newCarrierHz, uint32_t sampleRateHz) {
  if (newCarrierHz < 0.0f) {
    newCarrierHz = 0.0f;
  }

  carrierHz = newCarrierHz;

  // phaseInc determines how far the oscillator advances per audio sample
  phaseInc = (uint32_t)((carrierHz * 4294967296.0) / (double)sampleRateHz);
}

void ringModSetMix(float dry, float wet) {
  dryQ15 = clampQ15Gain(dry);
  wetQ15 = clampQ15Gain(wet);
}

int ringModProcessSample(int input) {
  phase += phaseInc;

  // Bipolar triangle carrier, approximately -1.0 to +1.0 in Q15
  int16_t carrier = triangleCarrierQ15(phase);

  // Ring modulation
  // input is centered audio, roughly -2048 to +2047
  // carrier is Q15, roughly -32767 to +32767
  // Shift right by 15 to convert back from Q15 (divides by 2^15)
  int32_t ring = ((int32_t)input * (int32_t)carrier) >> 15;

  // Mix dry original signal with wet ring-modulated signal
  int32_t mixed =
      ((int32_t)input * dryQ15) +
      ((int32_t)ring  * wetQ15);

  mixed = mixed >> 15;

  // clamp to centered 12-bit audio range
  if (mixed > 2047) mixed = 2047;
  if (mixed < -2048) mixed = -2048;

  return (int)mixed;
}