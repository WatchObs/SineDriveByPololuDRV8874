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
#include <Encoder.h>

// --- Host step loss timeout ---
const uint32_t LOSS_TIMEOUT_MS = 500;   // adjust as needed
volatile uint32_t lastHostStepTime = 0;
volatile bool hostTimedOut = false;

// How close (in degrees) the sine phase must be to a cardinal axis
// before switching to 8-step mode. Tune as needed.
#define EIGHTSTEP_ALIGN_TOL_DEG   3.0f

// Pins
const int PIN_A_PH   = 8;   // Phase (direction) for winding A
const int PIN_A_EN   = 3;   // Enable (PWM) for winding A
const int PIN_B_PH   = 4;   // Phase (direction) for winding B
const int PIN_B_EN   = 2;   // Enable (PWM) for winding B
const int PIN_LED    = 13;

const int BPWM_FREQ = 12000;

// Current sense (CS) pins from DRV8874 boards
const int PIN_CS_A = 16; // DRV8874 CS for winding A
const int PIN_CS_B = 15; // DRV8874 CS for winding B

const int PIN_HOST_ENABLE  = 20;
const int PIN_HOST_DIR     = 21;
const int PIN_HOST_FREQMSR = 22;  // Host requested rate (freq) used by FreqMeasure API

const int PIN_ISR_TOGGLE   = 10;  // scope probe (test signal output for logic analyzer)

// Waveform / motor geometry
const uint32_t uStepPerTurn = 51200;          // Microsteps per revolution
const int   cntLUT          = 1024;           // Table entries per electrical cycle
const float magCyclesPerRev = 50.f;           // Electrical cycles per mechanical revolution
const float phaseConv       = cntLUT / 360.f; // Convert magnetic phase angle to LUT index
const int   PWM_BITS        = 12;             // PWM resolution
const float PWM_MAG         = (float)(pow(2, PWM_BITS)-0); // PWM magnitude

const int      PIN_RATE_PWM     = 11;             // fixed hardware PWM test source
const uint32_t RATE_PWM_FREQ_HZ = uStepPerTurn/240; // FOR TESTING
const float    transFreqHz      = uStepPerTurn*2; // Frequency for sine<>square transition
bool           pendingEightStep = false;

// ISR timing
const uint32_t ISR_FREQ_HZ   = 25000UL; // ISR ticks per second
const uint32_t ISR_PERIOD_US = 1000000UL / ISR_FREQ_HZ;

// Host rate steps/sec conversion to magnetic phase
const float hostRTconv = 360.0/(ISR_FREQ_HZ * uStepPerTurn);

// ADC
const int ADC_BITS = 12;

// --- Shared state (volatile for ISR/main) ---
volatile float  demandAngle   = 0.0f;   // Demanded shaft angle (degrees)
volatile float  phaseA_f      = 0.0f;   // Magnetic angle (degrees)
volatile float  phaseAdv      = 0.0f;   // Phase advance
volatile float  pE            = 0.0f;   // Position error
volatile float  pEI           = 0.0f;   // Position error integration
volatile float  dirSign       = 1;      // +1 CW, -1 CCW
volatile int8_t enableDrv     = HIGH;   // Drive enable
volatile int    step8A        = 0;      // 8th mode step number phase A
volatile int    step8B        = 0;      // 8th mode step number phase B
volatile int    releaseForUse = 0;      // Stepper released for normal drive
volatile float  freqMeasureHz = 0.f;    // Host step frequency
volatile unsigned long freqMeasure = 0; // Host step counts per second (rate)

// Limit dutyCyc to protect DRV8874 (max 2.1A)
const float    PWM_GAIN_MAX = 1.0f;     // Empirical safe max (adjust as needed)
const float    PWM_GAIN_MIN = 0.0f;     // Empirical minimum (adjust to achieve motor rating)
volatile float dutyA        = 0.0f;     // Duty cycle phase A
volatile float dutyB        = 0.0f;     // Duty cycle phase 

// Slew-limited output frequency for phase integration
volatile float slewFreqHz = 0.f;        // deg/sec
const float SLEW_RATE_HZ_PER_ISR = (float)uStepPerTurn * 2.5f / (float)ISR_FREQ_HZ;

// Driver motor currents
const    float convCS   = (3.3f/(float)pow(2,ADC_BITS)) * (1.0f/1.1f); // CS scaling: 1.1V/A
volatile float currAAvg = 0.f;  // Filtered current A
volatile float currBAvg = 0.f;  // Filtered current B
volatile float currA    = 0.f;  // Motor coil current A
volatile float currB    = 0.f;  // Motor coil current B
volatile float currD    = 0.f;  // Rotor frame current - D axis
volatile float currQ    = 0.f;  // Rotor frame current - Q axis
volatile float currDReq = 0.0f; // Requested rotor frame current - D axis
volatile float currQReq = 0.0f; // Requested rotor frame current - Q axis
volatile float currPolA = 1.f;  // Polarity of current A
volatile float currPolB = 1.f;  // Polarity of current B
volatile float dutyD    = 0.f;  // Duty cycle D 
volatile float dutyQ    = 0.f;  // Duty cycle Q

// --- Mode tracking for LUT switching ---
enum DriveMode : uint8_t 
{
  MODE_SINE = 0,
  MODE_EIGHTH = 1
};

volatile DriveMode currentMode = MODE_SINE;

// Sine LUT
static float sineLUT[cntLUT];
// 8-step square wave direction and level
// Step index: 0..7 corresponds to 0°,45°,90°,135°,180°,225°,270°,315° of phaseA
static const int8_t eighthStepAct[8] = { LOW, HIGH, HIGH, HIGH, LOW, HIGH, HIGH, HIGH };
static const int8_t eighthStepDir[8] = {HIGH, HIGH, HIGH, HIGH, LOW,  LOW,  LOW,  LOW };

// Teensy 4.x hardware encoder pins
const int PIN_ENC_A   = 1; // Quadrature A
const int PIN_ENC_B   = 0; // Quadrature B
const int PIN_ENC_IDX = 9; // Index (Z) -- changed from 24 to 9

#define ENC_CNTS 20

Encoder motorEncoder(PIN_ENC_A, PIN_ENC_B);
volatile long  encCnt[ENC_CNTS]  = {0};
volatile int   encoderIndex  = -1;
volatile float encDeg        = 0.f;
volatile float encoderOffset = 0.f;  // Encoder offset (to magnetic cycle)
volatile float encCntDiff    = 0.f;  // Encoder difference (for velocity)
volatile float commDeg       = 0.f;  // Commutation angle
static int     encQuadRez    = 2500 * 4; 
volatile float motorVel      = 0.f;  // Motor velocity

static void setupLUTs()
{
  for (int n = 0; n < cntLUT; ++n)
  {
    float a = sinf((float)n * 2.0f * M_PI / (float)cntLUT);
    sineLUT[n] = a; // * PWM_MAG;
  }
}

void dutyDRV8874(float val, int PIN_PH, int PIN_EN)
{
  val *= PWM_MAG;

  if (val < 0.) 
  {
    digitalWrite(PIN_PH, LOW); // Reverse
    if (val <= -PWM_MAG) 
    {
      pinMode(PIN_EN, OUTPUT);
      digitalWriteFast(PIN_EN, HIGH);
    }
    else
    {
      analogWriteFrequency(PIN_EN, BPWM_FREQ);
      analogWrite(PIN_EN, (uint16_t)(-val));
    }
  } 
  else 
  {
    digitalWrite(PIN_PH, HIGH); // Forward
    if (val >= PWM_MAG) 
    {
      pinMode(PIN_EN, OUTPUT);
      digitalWriteFast(PIN_EN, HIGH);
    }
    else
    {
      analogWriteFrequency(PIN_EN, BPWM_FREQ);
      analogWrite(PIN_EN, (uint16_t)val);
    }
  }
}

void encoderIndexISR() 
{
  motorEncoder.write(0); // Zero the encoder position on index pulse
  encCnt[0] = 0;         // Zero the tracked position
  encoderIndex++;
}

// --- ISR: measurement + integration ---
void stepISR()
{
  digitalWriteFast(PIN_ISR_TOGGLE, HIGH);  // Debug

  // Motor currents
  currAAvg += (analogRead(PIN_CS_A) * convCS - currAAvg) * 0.1f;
  currBAvg += (analogRead(PIN_CS_B) * convCS - currBAvg) * 0.1f;

  // Apply sign to measured current
  currA = currAAvg * currPolA;
  currB = currBAvg * currPolB;

  // Encoder feedback
  for (int n=ENC_CNTS-2; n>=0;n--) encCnt[n+1] = encCnt[n];
  encCnt[0] = motorEncoder.read() % encQuadRez;
  encDeg = -(float)encCnt[0] * (360.0f / (float)encQuadRez) - encoderOffset;

  encCntDiff = ((encCnt[0] - encCnt[ENC_CNTS-1])/ENC_CNTS) * 360.f / encQuadRez;
  motorVel += ((float)encCntDiff * ISR_FREQ_HZ - motorVel) * 0.01f;

  float temp = encDeg * magCyclesPerRev * (1.f/360.f);
  commDeg = (temp - (int)temp) * 2 * M_PI;

  // d-q currents
  currD =  cosf(commDeg) * currA + sinf(commDeg) * currB;
  currQ = -sinf(commDeg) * currA + cosf(commDeg) * currB;

  // Signal frequency (commanded rate and direction)
  if (releaseForUse)
  {
    // 1) Sample host step frequency, direction and enable
    enableDrv = digitalReadFast(PIN_HOST_ENABLE) ? LOW : HIGH;
    dirSign   = digitalReadFast(PIN_HOST_DIR   ) ? 1.f : -1.f;
   
    if (FreqMeasure.available())
    {
      freqMeasure   = FreqMeasure.read();
      freqMeasureHz = FreqMeasure.countToFrequency(freqMeasure);
   
      // Host is alive: record activity and clear timeout
      lastHostStepTime = millis();
      hostTimedOut = false;
    }
   
    // --- Host timeout-aware target frequency ---
    if (hostTimedOut || (enableDrv == LOW)) freqMeasureHz = 0.0f; // Force a controlled ramp-down
   
    // Slew output frequency toward input frequency
    float freqDelta = freqMeasureHz - slewFreqHz;
    if      (freqDelta >  SLEW_RATE_HZ_PER_ISR) slewFreqHz += SLEW_RATE_HZ_PER_ISR;
    else if (freqDelta < -SLEW_RATE_HZ_PER_ISR) slewFreqHz -= SLEW_RATE_HZ_PER_ISR;
    else                                        slewFreqHz  = freqMeasureHz;
  }  // releaseForUse

  // 2) Phase integration from slew-limited rate
  static int servoDelay = 0;

  if (slewFreqHz > 0.f)
  {
    float Rate = slewFreqHz * hostRTconv;  // degrees/iteration

    if (releaseForUse)
    {
      int LowSpeed = (slewFreqHz < (uStepPerTurn>>4));

      if (1 || LowSpeed)
        demandAngle += dirSign * Rate;
      else
      {
        demandAngle = encDeg + dirSign * Rate;
        servoDelay  = 0;
      }

      if      (demandAngle >= 360.f) demandAngle -= 360.f;
      else if (demandAngle <    0.f) demandAngle += 360.f;

      if (servoDelay < ((int)ISR_FREQ_HZ*4))  // Transition to close loop
        servoDelay++;
      else if (LowSpeed)   // Low speed
      {
        pE = demandAngle - encDeg;
        if      (pE >=  180.f) pE -= 360.f;
        else if (pE <= -180.f) pE += 360.f;
  
        pEI += pE * 0.0001f;
      }
      else   // High speed
      {
        pE  = 0.f;
        pEI = 0.f;
      }
    }
    else
    {
      pE         = 0.f;
      servoDelay = 0;
      demandAngle += dirSign * Rate;
      if      (demandAngle >= 360.f) demandAngle -= 360.f;
      else if (demandAngle <    0.f) demandAngle += 360.f;
    }

    const float SERVO_KP  = 0.0f; // Position error gain
    const float SERVO_KPI = 0.0f; // Integrated position error gain

    float phase = (demandAngle + SERVO_KP * pE + SERVO_KPI * pEI) * magCyclesPerRev * (1.f/360.f);
    phaseAdv = dirSign * slewFreqHz * 0.0000f;
    phaseA_f = (phase - (int)phase) * 360.f + phaseAdv;  // + phase advance
  }
  else
  {
    servoDelay  = 0;
    demandAngle = encDeg;
    pE  = 0.f;
    pEI = 0.f;
  }

  if (releaseForUse)
  {
    // --- Mode selection based on slewed frequency ---
    if ((slewFreqHz > transFreqHz) && (currentMode == MODE_SINE))
      pendingEightStep = true;   // request transition
      // stay in sine mode until alignment occurs
    else if ((slewFreqHz < transFreqHz) && (currentMode == MODE_EIGHTH))
    {
      phaseA_f = step8A * 45.0f;
      currentMode = MODE_SINE;
    }
  } // releaseForUse

  // 3) Compute channel B (quarter period ahead) after any possible snap
//  float phaseB_f = phaseA_f + 90.f;
//  if (phaseB_f >= 360.f) phaseB_f -= 360.f;

  // 4) Lookup waveform values based on current mode
  // --- Safe disable when timed out and fully stopped ---
  if ((slewFreqHz < 1.f) && releaseForUse)
  {
    // Motor has fully ramped down; now it's safe to brake
    pinMode(PIN_A_EN, OUTPUT);
    digitalWriteFast(PIN_A_EN, LOW);
    pinMode(PIN_B_EN, OUTPUT);
    digitalWriteFast(PIN_B_EN, LOW);
  }
  else if (currentMode == MODE_SINE)
  {
    // Sine mode: use LUT
//    int idxA = (int)(phaseA_f * phaseConv);
//    int idxB = (int)(phaseB_f * phaseConv);

//    idxA %= cntLUT;
//    idxB %= cntLUT;
//    if (idxA < 0) idxA += cntLUT;
//    if (idxB < 0) idxB += cntLUT;

    // Fade PWM levels to control current
    float magAngle = 0.f;

    if (releaseForUse) 
    {
      currDReq = 0.0f;
      currQReq = 0.5f;

      dutyD  = 0.0;
      dutyQ  = 1.0f; //+= (currQReq - currQ) * 0.0001f;
      if      (dutyQ > PWM_GAIN_MAX) dutyQ = PWM_GAIN_MAX;
      else if (dutyQ < PWM_GAIN_MIN) dutyQ = PWM_GAIN_MIN;

      magAngle = commDeg + M_PI * .075f;  // Radians
    }
    else
    {
      dutyD = 0.15;
      dutyQ = 0.f;
//    dutyA = sineLUT[idxA] * dutyCyc;
//    dutyB = sineLUT[idxB] * dutyCyc;
      magAngle = phaseA_f * (M_PI / 180.f);  // Convert to Radians
    }

//  magAngle = phaseA_f * (M_PI / 180.f);
    
    dutyA = cosf(magAngle) * dutyD - sinf(magAngle) * dutyQ;
    dutyB = sinf(magAngle) * dutyD + cosf(magAngle) * dutyQ; 

    currPolA = (dutyA < 0.f) ? -1. : 1.;
    currPolB = (dutyB < 0.f) ? -1. : 1.;

    // DRV8874 PH/EN mode: set phase and PWM for each winding
    dutyDRV8874(dutyA, PIN_A_PH, PIN_A_EN);    
    dutyDRV8874(dutyB, PIN_B_PH, PIN_B_EN);   
 
    // --- Handle sine → 8-step transition alignment ---
    if (pendingEightStep)
    {
      // Switch mode when aligned with 180°
      if (fabsf(phaseA_f - 180.f) < EIGHTSTEP_ALIGN_TOL_DEG)
      {
        currentMode = MODE_EIGHTH;
        pendingEightStep = false;
      }
    }    
  }
  else if (currentMode == MODE_EIGHTH) 
  {
    // True 8-step square: derive step index from phaseA
    float phaseNorm = phaseA_f;
    while (phaseNorm >= 360.f) phaseNorm -= 360.f;
    while (phaseNorm <    0.f) phaseNorm += 360.f;

    step8A = (int)(phaseNorm / 45.0f) & 7;
    step8B = (step8A + 2) & 7;

    // DRV8874 PH/EN mode:
    pinMode(PIN_A_EN, OUTPUT);
    digitalWriteFast(PIN_A_EN, eighthStepAct[step8A]);
    digitalWriteFast(PIN_A_PH, eighthStepDir[step8A]);

    pinMode(PIN_B_EN, OUTPUT);
    digitalWriteFast(PIN_B_EN, eighthStepAct[step8B]);
    digitalWriteFast(PIN_B_PH, eighthStepDir[step8B]);

    currPolA = (eighthStepDir[step8A] == HIGH) ? 1.f : -1.f;
    currPolB = (eighthStepDir[step8B] == HIGH) ? 1.f : -1.f;
  } 

  digitalWrite(PIN_LED, enableDrv);       // Debug
  digitalWriteFast(PIN_ISR_TOGGLE, LOW);  // Debug
}
 
// IntervalTimer
IntervalTimer stepTimer;

void setup()
{
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}
  Serial.println("Sinusoidal micro-stepper (DRV8874 PH/EN mode, true 8-step square)");

  // pins
  pinMode(PIN_A_PH,        OUTPUT);
  pinMode(PIN_A_EN,        OUTPUT);
  pinMode(PIN_B_PH,        OUTPUT);
  pinMode(PIN_B_EN,        OUTPUT);
  pinMode(PIN_LED,         OUTPUT);
  pinMode(PIN_HOST_ENABLE, INPUT_PULLUP);
  pinMode(PIN_HOST_DIR,    INPUT_PULLUP);
  pinMode(PIN_ISR_TOGGLE,  OUTPUT);
  pinMode(PIN_ENC_IDX,     INPUT_PULLUP);

  digitalWriteFast(PIN_ISR_TOGGLE, LOW);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_IDX), encoderIndexISR, RISING);

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
  dirSign = 1.0f;

  setupLUTs();

  // Setup frequency counter/measure
  FreqMeasure.begin();

  // start ISR timer
  stepTimer.begin(stepISR, ISR_PERIOD_US);
}

void loop() 
{
  static int      operStep    = 0;
  static uint32_t magPoleTime = 0;
  static uint32_t lastDebugMs = 0;

  switch (operStep)
  {
    case 0:
      Serial.println("Index search");
//    currentMode == MODE_SINE;
//    encoderIndex = -1;
      releaseForUse = 0;
      enableDrv     = HIGH;
      dirSign       = -1.0f;
      operStep      = (encoderIndex >= 0) ? 1 : 2;
      slewFreqHz    = uStepPerTurn * .1;  // 10 seconds for complete revolution
      break;

    case 1:
      // On index - move away from it
      Serial.println("On index, moving off index");
      operStep++;
      break;

    case 2:
      // Initialization - find encoder index
      if (encoderIndex >= 0)        // Ensure index is found
      {
        slewFreqHz  = 0.f;
        magPoleTime = millis();
        Serial.println("Found index, moving to magnetic pole");
        operStep++;
      }
      break;

    case 3:
      // Force rotor to one pole
      phaseA_f = 0.f;
      if ((millis() - magPoleTime) > 2000)
      {
        Serial.print("Magnetic pole found at ");
        Serial.println(encDeg);
        encoderOffset = encDeg;
        demandAngle   = 0.f;
        releaseForUse = 1;
        operStep++;
      }
      break;

    case 4:
      if (1 && (millis() - lastDebugMs >= 100))
      {
//      static int pIndex = 0;
      
//      Serial.print(" freqMeasureHz="); Serial.print(freqMeasureHz);
//      Serial.print(" slewFreqHz=");    Serial.print(slewFreqHz);
//      Serial.print(" CS_A=");          Serial.print(currAAvg);
//      Serial.print(" CS_B=");          Serial.print(currBAvg);
//      Serial.print(" currentMode=");   Serial.print(currentMode);
        Serial.print(" commDeg=");       Serial.print(commDeg);
//      Serial.print(" dutyA=");         Serial.print(dutyA);
//      Serial.print(" dutyB=");         Serial.print(dutyB);
//      Serial.print(" currA=");         Serial.print(currA);
//      Serial.print(" currB=");         Serial.print(currB);
        Serial.print(" dutyD=");         Serial.print(dutyD);
        Serial.print(" dutyQ=");         Serial.print(dutyQ);
        Serial.print(" currD=");         Serial.print(currD);
        Serial.print(" currQ=");         Serial.print(currQ);
//      Serial.print(" currPolA=");      Serial.print(currPolA);
//      Serial.print(" currPolB=");      Serial.print(currPolB);
//      Serial.print(" dutyCyc=");       Serial.print(dutyCyc);
//      Serial.print(" encCntDiff=");    Serial.print(encCntDiff);
//      Serial.print(" motorVel=");      Serial.print(motorVel);
//      Serial.print(" encDeg=");        Serial.print(encDeg);
//      Serial.print(" DemAngle=");      Serial.print(demandAngle);
//      Serial.print(" phaseA_f=");      Serial.print(phaseA_f);
//      Serial.print(" step8A=");        Serial.print(step8A);
//      Serial.print(" step8B=");        Serial.print(step8B);
//      Serial.print(" phaseAdv=");      Serial.print(phaseAdv);
//      Serial.print(" pE=");            Serial.print(pE);
//      Serial.print(" pEI=");           Serial.print(pEI);
//      if (encoderIndex != pIndex) { Serial.print(" (index seen)"); pIndex = encoderIndex; } 
        Serial.println();
      
        lastDebugMs = millis();
      }
      break;
  }

  // --- Host step loss detection ---
  if (!hostTimedOut && ((millis() - lastHostStepTime) > LOSS_TIMEOUT_MS)) hostTimedOut = true;

  delay(1);
}
