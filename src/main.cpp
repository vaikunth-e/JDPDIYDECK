#include <Arduino.h>
#include <HardwareTimer.h>
#include "stm32f3xx_hal.h" // stm32 functionality for ADC & DAC

#include "reverb_model.h" // reverb functionality
#include "ringmod.h" // ring modulation

// Hardware mapping
// Input: A0 / PA0 = ADC1_IN1, Analog input
// Output: A2 / PA4 = DAC_OUT1, Analog output
// See circuit schematic for wiring and passive components
// Audio input should be AC-coupled and biased midpoint of supply voltage rabnge
// Audio output should be AC-coupled into the PCB's Audio_In input.

// Audio settings
// USing 32kHz, possible improvement? Sample rate of 16 kHz confirmed to provide decent audio; 8kHz is pretty noisy
// ADC uses 12-bit integer representation. Its integer range is thus 0 - 4095, and its voltage range is 0 - 3.3V, so 2048 is ~1.65V
// Biasing the AC audio to ADC midpoint is important because AC has negative voltages, while the ADC/DAC is unipolar and only handles 0 - 3.3V
static const uint32_t SAMPLE_RATE = 44100;
static const int ADC_MID = 2048;

// Increase gain to improve (NOT NECESSARY, old)
static const int GAIN_NUM = 1;
static const int GAIN_DEN = 1;

static const uint8_t REVERB_BUTTON = D4;
static const uint8_t RING_BUTTON = D2;
volatile bool reverbEnabled = false;
volatile bool ringEnabled = false;
volatile bool reverbPrintPending = false;
volatile bool reverbPrintState = false;
volatile bool ringPrintPending = false;
volatile bool ringPrintState = false;

static const float REVERB_WET = 0.45f;
static const float REVERB_DRY = 0.90f;
static const float REVERB_ROOMSIZE = 0.69f;
static const float REVERB_DAMPING = 0.20f;

// Ring modulation settings
static const float RING_CARRIER_HZ = 800.0f;
static const float RING_DRY = 0.70f;
static const float RING_WET = 0.60f;

static uint32_t ringPhase = 0;
static uint32_t ringPhaseInc = 0;

// Hardware timer enforces precise timing which is good for audio
// Timer/audio object's state
HardwareTimer *audioTimer = nullptr;

volatile uint32_t sampleCount = 0; // debug
volatile uint32_t clipLowCount = 0;
volatile uint32_t clipHighCount = 0;

volatile int dcEstimate = ADC_MID; // initial estimate of DC physical input bias, will be adjusted w/ MA depending on specific characteristics

// DAC: PA4 / DAC_OUT1
static inline void writeDAC(uint16_t value) {
  if (value > 4095) value = 4095;
  DAC1->DHR12R1 = value;
}

void setupDAC() {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_DAC1_CLK_ENABLE();

  // PA4 analog mode
  GPIOA->MODER |= GPIO_MODER_MODER4;
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR4;

  // Reset & enable DAC channel 1 (PA4)
  DAC1->CR = 0;
  DAC1->CR |= DAC_CR_EN1;

  // Midpoint output, 2048 is about 1.65 V (see above)
  writeDAC(2048);
}

// ADC: PA0 / ADC1_IN1
void setupADC() {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_ADC12_CLK_ENABLE();

  // PA0 analog mode, no pull-up/pull-down
  GPIOA->MODER |= GPIO_MODER_MODER0;
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR0;

  // Give ADC12 a synchronous clock from HCLK
  // This is important; calibration may never finish
  ADC12_COMMON->CCR &= ~ADC12_CCR_CKMODE;
  ADC12_COMMON->CCR |= ADC12_CCR_CKMODE_0;   // HCLK / 1

  // ADC is disabled before calibration/configuration.
  if (ADC1->CR & ADC_CR_ADEN) {
    ADC1->CR |= ADC_CR_ADDIS;
    while (ADC1->CR & ADC_CR_ADEN) {
      // wait
    }
  }

  // clear ADC ready flag
  ADC1->ISR |= ADC_ISR_ADRDY;

  // enable ADC's voltage regulator
  ADC1->CR &= ~ADC_CR_ADVREGEN;
  ADC1->CR |= ADC_CR_ADVREGEN_0;

  // delay for ADC's voltage regulator startup
  delay(10);

  // single ended calibration
  ADC1->CR &= ~ADC_CR_ADCALDIF;
  ADC1->CR |= ADC_CR_ADCAL;

  uint32_t t0 = millis();
  while (ADC1->CR & ADC_CR_ADCAL) {
    if (millis() - t0 > 1000) {
      Serial.println("FAILED: ADC calibration timeout"); // debug
      while (1) {}
    }
  }

  // Configure ADC:
  // 12 bit resolution, right aligned, single conversion, software trigger
  ADC1->CFGR = 0;

  // Long sample time for stability with bias network
  ADC1->SMPR1 &= ~ADC_SMPR1_SMP1;
  ADC1->SMPR1 |= ADC_SMPR1_SMP1;

  // Regular sequence length = 1 conversion
  ADC1->SQR1 = 0;

  // First conversion = channel 1, which is PA0 / ADC1_IN1
  ADC1->SQR1 |= (1U << ADC_SQR1_SQ1_Pos);

  // Enable ADC
  ADC1->CR |= ADC_CR_ADEN;

  t0 = millis();
  while (!(ADC1->ISR & ADC_ISR_ADRDY)) {
    if (millis() - t0 > 1000) {
      Serial.println("FAILED: ADC ready timeout"); // debug
      while (1) {}
    }
  }
}

static inline uint16_t readADC() {
  // Start one software conversion
  ADC1->CR |= ADC_CR_ADSTART;

  // Wait for conversion complete
  while (!(ADC1->ISR & ADC_ISR_EOC)) {
    // wait
  }

  return (uint16_t)(ADC1->DR & 0x0FFF);
}

// Audio ISR
// Direct ADC read + direct DAC write.
// at 16 kHz, this only has 62.5 us to complete work, so avoid slowdowns (Arduino r/w, Serial print)
void audioISR() {
  int sample = readADC();

  // Remove bottom two noisy bits
  sample = sample & ~0x03;

  // Slowly track actual DC bias
  dcEstimate = dcEstimate + ((sample - dcEstimate) >> 8); // moving average

  // Convert biased ADC sample into centered audio by removing estimated system DC bias
  // centers at 0
  int centered = sample - dcEstimate;

  // Optional gain
  centered = (centered * GAIN_NUM) / GAIN_DEN; // unnecessary

  // Shift back to DAC midpoint (ADC_MID = 2048 = 1.65V)
  // Reverb functionality added if reverb enabled
  int processed = centered;

  if (reverbEnabled) {
    processed = reverbProcessSample(processed);
  } 
  if (ringEnabled) {
    processed = ringModProcessSample(processed);
  }

  int out = processed + ADC_MID;

  if (out < 0) {
    out = 0;
    clipLowCount++; // debug, check for too low
  }

  if (out > 4095) {
    out = 4095;
    clipHighCount++; // debug, check for too high
  }

  writeDAC((uint16_t)out); // write computed output to PCB Audio_In with the DAC

  sampleCount++; // debug
}

void reverbButtonISR() {
  static volatile uint32_t lastPressUs = 0;

  uint32_t now = micros();

  if ((uint32_t)(now - lastPressUs) > 250000UL) {
    reverbEnabled = !reverbEnabled;

    reverbPrintState = reverbEnabled;
    reverbPrintPending = true;

    lastPressUs = now;
  }
}

void ringButtonISR() {
  static volatile uint32_t lastPressUs = 0;

  uint32_t now = micros();

  if ((uint32_t)(now - lastPressUs) > 250000UL) {
    ringEnabled = !ringEnabled;

    ringPrintState = ringEnabled;
    ringPrintPending = true;

    lastPressUs = now;
  }
}

void setup() {
  Serial.setTx(PA2);
  Serial.setRx(PA3);

  Serial.begin(115200); // !!!!!!!!!!
  delay(1000); 

  Serial.println("Starting..."); // debug
  Serial.flush();

  setupADC();
  setupDAC();

  pinMode(REVERB_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(REVERB_BUTTON), reverbButtonISR, FALLING);

  pinMode(RING_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RING_BUTTON), ringButtonISR, FALLING);

  // initialize reverb before enabling the audio interrupt
  reverbInit(SAMPLE_RATE);
  reverbSetWet(REVERB_WET);
  reverbSetDry(REVERB_DRY);
  reverbSetRoomSize(REVERB_ROOMSIZE);
  reverbSetDamping(REVERB_DAMPING);

  // initialize ring!
  ringModInit(SAMPLE_RATE);
  ringModSetCarrierHz(RING_CARRIER_HZ, SAMPLE_RATE);
  ringModSetMix(RING_DRY, RING_WET);

  audioTimer = new HardwareTimer(TIM3); // dont use TIM2
  audioTimer->setOverflow(SAMPLE_RATE, HERTZ_FORMAT);
  audioTimer->attachInterrupt(audioISR);
  audioTimer->resume();
}

void loop() {
  if (reverbPrintPending) {
    noInterrupts();
    bool state = reverbPrintState;
    reverbPrintPending = false;
    interrupts();

    Serial.print("Reverb: ");
    Serial.println(state ? "ON" : "OFF");
  }

  if (ringPrintPending) {
    noInterrupts();
    bool state = ringPrintState;
    ringPrintPending = false;
    interrupts();

    Serial.print("Ring modulation: ");
    Serial.println(state ? "ON" : "OFF");
  }

  static uint32_t lastPrint = 0;

  if (millis() - lastPrint >= 1000) { // debug
    lastPrint = millis();

    noInterrupts();

    uint32_t samples = sampleCount;
    uint32_t clipsLow = clipLowCount; // debug
    uint32_t clipsHigh = clipHighCount;
    int dc = dcEstimate;

    sampleCount = 0; // debug
    clipLowCount = 0;
    clipHighCount = 0;

    interrupts();

    /*
    Serial.print("DC estimate: "); // debugg
    Serial.print(dc);

    Serial.print(" | samples/sec: ");
    Serial.print(samples);

    Serial.print(" | clip low: ");
    Serial.print(clipsLow);

    Serial.print(" | clip high: ");
    Serial.println(clipsHigh);
    */
  }
}