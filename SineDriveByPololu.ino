/*
  Sinusoidal micro stepper with loss-of-pulses safety
  - External STEP on pin 21 used only for measurement (freq)
  - ISR integrates published host rate (indices per ISR) into a float phase accumulator
  - If no STEP edges seen for LOSS_TIMEOUT_MS, published rate is forced to zero
*/

#include <Arduino.h>
#include <stdint.h>
#include <math.h>
#include <FreqMeasure.h>

// Loss-of-pulses timeout (milliseconds)
const unsigned long LOSS_TIMEOUT_MS = 1000UL; // 1 second  TODO

// Pins
const int PIN_A_PH   = 4;   // Phase (direction) for winding A
const int PIN_A_EN   = 2;   // Enable (PWM) for winding A
const int PIN_B_PH   = 8;   // Phase (direction) for winding B
const int PIN_B_EN   = 3;   // Enable (PWM) for winding B
const int PIN_LED    = 13;

const int BPWM_FREQ = 15000;

// Current sense (CS) pins from DRV8874 boards
// Teensy 4.x: A1 = GPIO 15, A2 = GPIO 16
const int PIN_CS_A = A1; // DRV8874 CS for winding A (A1, GPIO 15)
const int PIN_CS_B = A2; // DRV8874 CS for winding B (A2, GPIO 16)

const int PIN_HOST_ENABLE  = 20;
const int PIN_HOST_DIR     = 21;
const int PIN_HOST_FREQMSR = 22;  // FreqMeasure API

const int PIN_RATE_PWM = 11;              // fixed hardware PWM test source
const uint32_t RATE_PWM_FREQ_HZ = 51200/240;  // FOR TESTING
const float transFreqHz = 50000.f;  // Frequency for sine<>square transition

const int PIN_ISR_TOGGLE = 10;  // scope probe (test signal output for logic analyzer)

// Waveform / motor geometry
const int   cntLUT          = 1024;              // Table entries per electrical cycle
const float cyclesPerRev    = 50;                // Electrical cycles per mechanical revolution
const float phaseConv       = cntLUT / 360.f;    // Convert magnetic phase angle to LUT index
const float microStepsPerRev= cntLUT * cyclesPerRev;
const int   PWM_BITS        = 8;                 // PWM resolution
const float PWM_MAG         = (float)(pow(2, PWM_BITS)-0); // PWM magnitude

// ISR timing
const uint32_t ISR_FREQ_HZ   = 25000UL; // ISR ticks per second
const uint32_t ISR_PERIOD_US = 1000000UL / ISR_FREQ_HZ;

// Host rate steps/sec conversion to magnetic phase
const float hostRTconv = 360.0/(ISR_FREQ_HZ * cntLUT);

// ADC
const int ADC_BITS = 12;

// --- Shared state (volatile for ISR/main) ---
volatile float  phaseA_f      = 0.0f;   // Magnetic angle (degrees)
volatile float  dirSign       = 1;      // +1 CW, -1 CCW
volatile int8_t enableDrv     = HIGH;   // Host enableDrv
volatile int8_t enableDrv_    = LOW;    // Host enableDrv inverse (bar)
volatile unsigned long freqMeasure   = 0;   // Host step counts per second (rate)
volatile float         freqMeasureHz = 0.f; // Host step frequency

// Limit pwmGain to protect DRV8874 (max 2.1A)
const float PWM_GAIN_MAX = 1.0f;        // Empirical safe max (adjust as needed)
const float PWM_GAIN_MIN = 0.3f;        // Empirical minimum (adjust to achieve motor rating)
volatile float pwmGain   = 0.3f;        // Gain on PWM amplitude

// Slew-limited output frequency for phase integration
volatile float slewFreqHz = 0.f;
const float SLEW_RATE_HZ_PER_SEC = 51200.f;
const float SLEW_RATE_HZ_PER_ISR = SLEW_RATE_HZ_PER_SEC / ISR_FREQ_HZ; // 2.048 Hz per ISR at 25kHz

// PWM outputs (written by ISR)
volatile uint16_t pwmA_pos = 0, pwmA_neg = 0, pwmB_pos = 0, pwmB_neg = 0;

// Driver motor currents
volatile float    avgCS_A = 0, avgCS_B = 0;
//const int   avgAdcCnt = 10;
//const float convCS = (3.3f/(float)pow(2,ADC_BITS)) * (1.0f/1.1f) * (1.0f/(float)avgAdcCnt); // CS scaling: 1.1V/A, avgAdcCnt samples
const float convCS = (3.3f/(float)pow(2,ADC_BITS)) * (1.0f/1.1f); // CS scaling: 1.1V/A

// --- Mode tracking for LUT switching ---
enum DriveMode : uint8_t {
  MODE_SINE = 0,
  MODE_EIGHTH = 1
};

volatile DriveMode currentMode = MODE_SINE;

// Sine LUT
static float sineLUT[cntLUT];
// 8-step square wave direction and level
// Step index: 0..7 corresponds to 0°,45°,90°,...,315° of phaseA
static const int8_t eighthStepActA[8] = { HIGH, HIGH,  LOW, HIGH, HIGH, HIGH, LOW,  HIGH };
static const int8_t eighthStepDirA[8] = {  LOW,  LOW,  LOW, HIGH, HIGH, HIGH, HIGH,  LOW };
static const int8_t eighthStepActB[8] = {  LOW, HIGH, HIGH, HIGH,  LOW, HIGH, HIGH, HIGH };
static const int8_t eighthStepDirB[8] = {  LOW, HIGH, HIGH, HIGH, HIGH,  LOW,  LOW,  LOW };

static void setupLUTs()
{
  for (int n = 0; n < cntLUT; ++n)
  {
    float a = sinf((float)n * 2.0f * M_PI / (float)cntLUT);
    sineLUT[n] = a * PWM_MAG;

    // You can comment these prints out once verified
    Serial.print("Angle=");      Serial.print(n*360/cntLUT);
    Serial.print(" Sine=");      Serial.print(sineLUT[n]);
    Serial.println();
  }
}

// Interpolated lookup using float index (currently unused but kept)
static inline float lookupInterpolatedFloat(float phase)
{
  float idx = phase * phaseConv;
  if (idx >= (float)cntLUT) idx -= floorf(idx / (float)cntLUT) * (float)cntLUT;
  if (idx < 0.0f)           idx += ceilf(-idx / (float)cntLUT) * (float)cntLUT;
  int   i    = (int)floorf(idx);
  float frac = idx - (float)i;
  int   j    = (i + 1 >= cntLUT) ? 0 : i + 1;
  float v0   = sineLUT[i];
  float v1   = sineLUT[j];
  float interp = v0 + (v1 - v0) * frac;
  return interp;
}

// Snap a phase (degrees) to the next 45° boundary in direction of motion
static inline float snapPhaseToEighth(float phaseDeg, float dirSign)
{
  // Normalize to [0, 360)
  while (phaseDeg >= 360.0f) phaseDeg -= 360.0f;
  while (phaseDeg <    0.0f) phaseDeg += 360.0f;

  const float stepDeg = 45.0f;
  float idx = phaseDeg / stepDeg;

  float snappedIdx;
  if (dirSign >= 0.0f) {
    // Moving "forward": snap to next or current higher step
    snappedIdx = ceilf(idx - 1e-6f);
  } else {
    // Moving "backward": snap to previous or current lower step
    snappedIdx = floorf(idx + 1e-6f);
  }

  if (snappedIdx >= 8.0f) snappedIdx -= 8.0f;
  if (snappedIdx <  0.0f) snappedIdx += 8.0f;

  return snappedIdx * stepDeg; // back to degrees
}

// IntervalTimer
IntervalTimer stepTimer;

// --- ISR: measurement + integration ---
void stepISR()
{
  static int subBandAdc = 0;

  digitalWriteFast(PIN_ISR_TOGGLE, HIGH);  // Debug

  // 1) Sample host step frequency, direction and enable
  enableDrv  = digitalReadFast(PIN_HOST_ENABLE) ? LOW : HIGH;
  enableDrv_ = (enableDrv == HIGH) ? LOW : HIGH;
  dirSign    = digitalReadFast(PIN_HOST_DIR) ? 1.0f : -1.0f;

  if (FreqMeasure.available())
  {
    freqMeasure   = FreqMeasure.read();
    freqMeasureHz = FreqMeasure.countToFrequency(freqMeasure);
  }

  // Slew output frequency toward input frequency
  float freqDelta = freqMeasureHz - slewFreqHz;
  if (freqDelta > SLEW_RATE_HZ_PER_ISR) {
    slewFreqHz += SLEW_RATE_HZ_PER_ISR;
  } else if (freqDelta < -SLEW_RATE_HZ_PER_ISR) {
    slewFreqHz -= SLEW_RATE_HZ_PER_ISR;
  } else {
    slewFreqHz = freqMeasureHz;
  }

  // 2) Phase integration from slew-limited rate
  float Rate = slewFreqHz * hostRTconv;  // degrees/sec
  if (enableDrv == HIGH)
  {
    phaseA_f += dirSign * Rate;
    if      (phaseA_f >= 360.f) phaseA_f -= 360.f;
    else if (phaseA_f <  0.0f)  phaseA_f += 360.f;
  }

  // --- Mode selection based on slewed frequency ---
  // You can add hysteresis here if you like
  DriveMode desiredMode = (slewFreqHz > transFreqHz) ? MODE_EIGHTH : MODE_SINE;

  // Detect mode transition
  if (desiredMode != currentMode) {
    // On transition into 8th-step, snap phase to a legal 45° boundary
    if (desiredMode == MODE_EIGHTH) {
      phaseA_f = snapPhaseToEighth(phaseA_f, dirSign);
    }
    // On transition back to sine, phaseA_f is already at a valid angle;
    // sine LUT will just use that exact phase.
    currentMode = desiredMode;
  }

  // 3) Compute channel B (quarter period ahead) after any possible snap
  float phaseB_f = phaseA_f + 90.f;
  if (phaseB_f >= 360.f) phaseB_f -= 360.f;

  // Fade PWM levels to control current
  pwmGain = PWM_GAIN_MIN + slewFreqHz * 0.00001f;
  if (pwmGain > PWM_GAIN_MAX) pwmGain = PWM_GAIN_MAX;

  // 4) Lookup waveform values based on current mode
  if (currentMode == MODE_EIGHTH) 
  {
    // True 8-step square: derive step index from phaseA
    float phaseNorm = phaseA_f;
    while (phaseNorm >= 360.0f) phaseNorm -= 360.0f;
    while (phaseNorm <   0.0f) phaseNorm += 360.0f;

    int step8 = (int)(phaseNorm / 45.0f) & 7;

    // DRV8874 PH/EN mode:
    pinMode(PIN_A_EN, OUTPUT);
    digitalWriteFast(PIN_A_EN, eighthStepActA[step8]);
    digitalWriteFast(PIN_A_PH, eighthStepDirA[step8]);

    pinMode(PIN_B_EN, OUTPUT);
    digitalWriteFast(PIN_B_EN, eighthStepActB[step8]);
    digitalWriteFast(PIN_B_PH, eighthStepDirB[step8]);
  } 
  else 
  {
    // Sine mode: use LUT
    int idxA = (int)(phaseA_f * phaseConv);
    int idxB = (int)(phaseB_f * phaseConv);

    idxA %= cntLUT;
    idxB %= cntLUT;
    if (idxA < 0) idxA += cntLUT;
    if (idxB < 0) idxB += cntLUT;

    float valA = sineLUT[idxA] * pwmGain;
    float valB = sineLUT[idxB] * pwmGain;

    // DRV8874 PH/EN mode: set phase and PWM for each winding
    if (valA < 0.) 
    {
      digitalWrite(PIN_A_PH, LOW); // Reverse
      if (valA <= -PWM_MAG) 
      {
        pinMode(PIN_A_EN, OUTPUT);
        digitalWriteFast(PIN_A_EN, HIGH);
      }
      else
      {
        analogWriteFrequency(PIN_A_EN, BPWM_FREQ);
        analogWrite(PIN_A_EN, (uint16_t)(-valA));
      }
    } 
    else 
    {
      digitalWrite(PIN_A_PH, HIGH); // Forward
      if (valA >= PWM_MAG) 
      {
        pinMode(PIN_A_EN, OUTPUT);
        digitalWriteFast(PIN_A_EN, HIGH);
      }
      else
      {
        analogWriteFrequency(PIN_A_EN, BPWM_FREQ);
        analogWrite(PIN_A_EN, (uint16_t)valA);
      }
    }

    if (valB < 0.) 
    {
      digitalWrite(PIN_B_PH, LOW);
      if (valB <= -PWM_MAG) 
      {
        pinMode(PIN_B_EN, OUTPUT);
        digitalWriteFast(PIN_B_EN, HIGH);
      }
      else
      {
        analogWriteFrequency(PIN_B_EN, BPWM_FREQ);
        analogWrite(PIN_B_EN, (uint16_t)(-valB));
      }
    } 
    else 
    {
      digitalWrite(PIN_B_PH, HIGH);
      if (valB >= PWM_MAG) 
      {
        pinMode(PIN_B_EN, OUTPUT);
        digitalWriteFast(PIN_B_EN, HIGH);
      }
      else
      {
        analogWriteFrequency(PIN_B_EN, BPWM_FREQ);
        analogWrite(PIN_B_EN, (uint16_t)valB);
      }
    }
  }

  // 5) ADC
  if (subBandAdc == 0)
  {
    avgCS_A += (analogRead(PIN_CS_A) * convCS - avgCS_A) * 0.01f;
    subBandAdc = 1;
  }
  else
  {
    avgCS_B += (analogRead(PIN_CS_B) * convCS - avgCS_B) * 0.01f;
    subBandAdc = 0;
  }

  digitalWrite(PIN_LED, enableDrv);       // Debug
  digitalWriteFast(PIN_ISR_TOGGLE, LOW);  // Debug
}

void setup()
{
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}
  Serial.println("Sinusoidal micro-stepper (DRV8874 PH/EN mode, true 8-step square)");

  // pins
  pinMode(PIN_A_PH, OUTPUT);
  pinMode(PIN_A_EN, OUTPUT);
  pinMode(PIN_B_PH, OUTPUT);
  pinMode(PIN_B_EN, OUTPUT);
  pinMode(PIN_LED, OUTPUT);

  pinMode(PIN_HOST_ENABLE, INPUT_PULLUP);
  pinMode(PIN_HOST_DIR, INPUT_PULLUP);

  pinMode(PIN_ISR_TOGGLE, OUTPUT);
  digitalWriteFast(PIN_ISR_TOGGLE, LOW);

  // Common to all PWMs
  analogWriteResolution(PWM_BITS);

  // TEST - Configure fixed hardware PWM on PIN_RATE_PWM once and never touch it again
  pinMode(PIN_RATE_PWM, OUTPUT);
  analogWriteFrequency(PIN_RATE_PWM, RATE_PWM_FREQ_HZ);
  analogWrite(PIN_RATE_PWM, (int)100); // set and leave

  // Configure PWM outputs for DRV8874
  analogWriteFrequency(PIN_A_EN, BPWM_FREQ);
  analogWriteFrequency(PIN_B_EN, BPWM_FREQ);

  analogReadResolution(ADC_BITS);
  analogReadAveraging(1);  // No averaging to speed up

  // Direction sign
  dirSign = 1;

  setupLUTs();

  // Setup frequency counter/measure
  FreqMeasure.begin();

  // start ISR timer
  stepTimer.begin(stepISR, ISR_PERIOD_US);
}

void loop() {
  static unsigned long lastDebugMs = 0;
  // 2 Hz debug printing
  if (1 && (millis() - lastDebugMs >= 500))
  {
//  Serial.print(" freqMeasureHz="); Serial.print(freqMeasureHz);
    Serial.print(" slewFreqHz=");    Serial.print(slewFreqHz);
    Serial.print(" CS_A=");          Serial.print(avgCS_A);
    Serial.print(" CS_B=");          Serial.print(avgCS_B);
    Serial.print(" mode=");
    Serial.print((currentMode == MODE_EIGHTH) ? "8step" : "sine");
    Serial.print(" pwmGain=");
    Serial.print((currentMode == MODE_EIGHTH) ? 1.0f : pwmGain);
    Serial.println();

    lastDebugMs = millis();
  }

  delay(1);
}
