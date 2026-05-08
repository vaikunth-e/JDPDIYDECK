#ifndef RING_MOD_H
#define RING_MOD_H

#include <Arduino.h>

// Initialize ring modulation state.
// Call once in setup() before the audio timer starts.
void ringModInit(uint32_t sampleRateHz);

// Process one centered audio sample.
// Input should be centered around 0, roughly -2048 to +2047.
// Output is also centered around 0.
int ringModProcessSample(int input);

// Change the carrier oscillator frequency.
// Example: 80 Hz = growly, 120 Hz = robotic, 400 Hz = metallic.
void ringModSetCarrierHz(float carrierHz, uint32_t sampleRateHz);

// Set dry/wet mix using normal float values from 0.0 to 1.0.
// These are converted internally to Q15 integer gains.
void ringModSetMix(float dry, float wet);

#endif