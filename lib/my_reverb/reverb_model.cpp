#include "reverb_model.h"
#include "reverb_tuning.h"
#include "reverb_comb.h"
#include "reverb_allpass.h"

namespace {

static inline float clamp01(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

// delay buffers in this file because model has the full reverb network
static float combBuf1[REVERB_COMB_TUNING_1];
static float combBuf2[REVERB_COMB_TUNING_2];
static float combBuf3[REVERB_COMB_TUNING_3];
static float combBuf4[REVERB_COMB_TUNING_4];
static float combBuf5[REVERB_COMB_TUNING_5];
static float combBuf6[REVERB_COMB_TUNING_6];
static float combBuf7[REVERB_COMB_TUNING_7];
static float combBuf8[REVERB_COMB_TUNING_8];

static float allpassBuf1[REVERB_ALLPASS_TUNING_1];
static float allpassBuf2[REVERB_ALLPASS_TUNING_2];
static float allpassBuf3[REVERB_ALLPASS_TUNING_3];
static float allpassBuf4[REVERB_ALLPASS_TUNING_4];

static ReverbComb combs[REVERB_NUM_COMBS];
static ReverbAllpass allpasses[REVERB_NUM_ALLPASSES];

static float wetLevel = 0.28f;
static float dryLevel = 0.85f;
static float roomSize = 0.70f;
static float damping = 0.35f;

void updateInternalParameters() {
  const float feedback = REVERB_ROOM_OFFSET + (REVERB_ROOM_SCALE * roomSize);
  const float damp = REVERB_DAMP_SCALE * damping;

  for (int i = 0; i < REVERB_NUM_COMBS; i++) {
    combs[i].setFeedback(feedback);
    combs[i].setDamping(damp);
  }

  for (int i = 0; i < REVERB_NUM_ALLPASSES; i++) {
    allpasses[i].setFeedback(REVERB_ALLPASS_FEEDBACK);
  }
}

} // namespace

void reverbInit(uint32_t sampleRateHz) {
  // Current delay lengths are statically tuned for 32 kHz
  (void)sampleRateHz;

  combs[0].setBuffer(combBuf1, REVERB_COMB_TUNING_1);
  combs[1].setBuffer(combBuf2, REVERB_COMB_TUNING_2);
  combs[2].setBuffer(combBuf3, REVERB_COMB_TUNING_3);
  combs[3].setBuffer(combBuf4, REVERB_COMB_TUNING_4);
  combs[4].setBuffer(combBuf5, REVERB_COMB_TUNING_5);
  combs[5].setBuffer(combBuf6, REVERB_COMB_TUNING_6);
  combs[6].setBuffer(combBuf7, REVERB_COMB_TUNING_7);
  combs[7].setBuffer(combBuf8, REVERB_COMB_TUNING_8);

  allpasses[0].setBuffer(allpassBuf1, REVERB_ALLPASS_TUNING_1);
  allpasses[1].setBuffer(allpassBuf2, REVERB_ALLPASS_TUNING_2);
  allpasses[2].setBuffer(allpassBuf3, REVERB_ALLPASS_TUNING_3);
  allpasses[3].setBuffer(allpassBuf4, REVERB_ALLPASS_TUNING_4);

  updateInternalParameters();
  reverbMute();
}

int reverbProcessSample(int centeredSample) {
  // Convert centered 12-bit ADC units to -1.0 to +1.0
  float input = (float)centeredSample / 2048.0f;

  float reverb = 0.0f;
  const float combInput = input * REVERB_FIXED_GAIN;

  // Schroeder pipeline input -> combs -> allpass to smaer -> wetness -> out

  // parallel comb filters create the decaying reflections Schroeder!
  for (int i = 0; i < REVERB_NUM_COMBS; i++) {
    reverb += combs[i].process(combInput);
  }

  // serial allpass filters diffuse/smear those reflections
  for (int i = 0; i < REVERB_NUM_ALLPASSES; i++) {
    reverb = allpasses[i].process(reverb);
  }

  float output = (dryLevel * input) + (wetLevel * reverb);

  // limiter before going back to centered DAC units
  if (output > 0.98f) output = 0.98f;
  if (output < -0.98f) output = -0.98f;

  return (int)(output * 2048.0f);
}

void reverbSetWet(float wet) {
  wetLevel = clamp01(wet);
}

void reverbSetDry(float dry) {
  dryLevel = clamp01(dry);
}

void reverbSetRoomSize(float value) {
  roomSize = clamp01(value);
  updateInternalParameters();
}

void reverbSetDamping(float value) {
  damping = clamp01(value);
  updateInternalParameters();
}

void reverbMute() {
  for (int i = 0; i < REVERB_NUM_COMBS; i++) {
    combs[i].mute();
  }

  for (int i = 0; i < REVERB_NUM_ALLPASSES; i++) {
    allpasses[i].mute();
  }
}
