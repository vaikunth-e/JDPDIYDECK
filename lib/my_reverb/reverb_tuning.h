#ifndef REVERB_TUNING_H
#define REVERB_TUNING_H

#include <Arduino.h>

// original mono/left delay lengths scaled to approximately 32 kHz.

static const uint32_t REVERB_NOMINAL_SAMPLE_RATE = 44100;

static const int REVERB_NUM_COMBS = 8;
static const int REVERB_NUM_ALLPASSES = 4;

static const int REVERB_COMB_TUNING_1 = 810;
static const int REVERB_COMB_TUNING_2 = 862;
static const int REVERB_COMB_TUNING_3 = 927;
static const int REVERB_COMB_TUNING_4 = 984;
static const int REVERB_COMB_TUNING_5 = 1032;
static const int REVERB_COMB_TUNING_6 = 1082;
static const int REVERB_COMB_TUNING_7 = 1130;
static const int REVERB_COMB_TUNING_8 = 1174;

static const int REVERB_ALLPASS_TUNING_1 = 404;
static const int REVERB_ALLPASS_TUNING_2 = 320;
static const int REVERB_ALLPASS_TUNING_3 = 248;
static const int REVERB_ALLPASS_TUNING_4 = 163;

// uses a small input gain before the parallel comb network to keep the
// feedback network controlled
static const float REVERB_FIXED_GAIN = 0.05f;

// parameter mapping, room/damp values are 0.0f - 1.0f.
static const float REVERB_ROOM_OFFSET = 0.70f;
static const float REVERB_ROOM_SCALE = 0.28f;
static const float REVERB_DAMP_SCALE = 0.40f;
static const float REVERB_ALLPASS_FEEDBACK = 0.50f;

#endif
