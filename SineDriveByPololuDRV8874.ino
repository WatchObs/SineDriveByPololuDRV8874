/*
  Sinusoidal micro stepper with loss-of-pulses safety
  - External STEP on pin 21 used only for measurement (freq)
  - ISR integrates published host rate (indices per ISR) into a float phase accumulator
  - If no STEP edges seen for LOSS_TIMEOUT_MS, published rate is forced to zero
*/

#include <Arduino.h>
#include <stdint.h>
#include <math.h>
#include <FreqMeasureMulti.h>
#include <Encoder.h>

// --- Host step loss timeout ---
const uint32_t LOSS_TIMEOUT_MS = 500;   // adjust as needed
volatile uint32_t lastHostStepTime = 0;
volatile bool hostTimedOut = false;

volatile float fMax = 0.f;
volatile float fMin = 0.f;

// Pins
const int PIN_A_PH         = 5;   // Phase (direction) for winding A
const int PIN_A_EN         = 3;   // Enable (PWM) for winding A
const int PIN_B_PH         = 4;   // Phase (direction) for winding B
const int PIN_B_EN         = 2;   // Enable (PWM) for winding B
const int PIN_CS_A         = 16;  // DRV8874 CS for winding A - current sensing
const int PIN_CS_B         = 15;  // DRV8874 CS for winding B - current sensing
const int PIN_ENC_A        = 0;   // Encoder Quadrature A
const int PIN_ENC_B        = 1;   // Encoder Quadrature B
const int PIN_ENC_IDX      = 12;  // Encoder Index (Z)
const int PIN_ENC_RATE     = 23;  // Measure encoder pulse rate (phase A) - pairs with 22
const int PIN_HOST_ENABLE  = 20;  // Host motor enable DIP
const int PIN_HOST_DIR     = 21;  // Host motor direction DIP
const int PIN_HOST_RATE    = 22;  // Host requested rate (freq) - pairs with 23
const int PIN_RX2          = 7;   // RS485 receive comm with host
const int PIN_TX2          = 8;   // RS485 transmit comm with host
const int PIN_DE_RE        = 6;   // RS485 bus enable
const int PIN_ISR_TOGGLE   = 10;  // Debug - ISR active signal
const int PIN_RATE_PWM     = 11;  // Debug - local motor rate
const int PIN_LED          = 13;

const uint32_t RS485_BAUD = 115200; // RS485 host comm baud rate

// Waveform / motor geometry
const int   BPWM_FREQ       = 25000;          // PWM frequency for DRV8874 driver
const uint32_t uStepPerTurn = 51200;          // Microsteps per revolution
const float maxFreq         = uStepPerTurn*3; // Maximum frequency (set according P/S & stepper specs)
const float transFreqHz     = uStepPerTurn/2; // Frequency for drive mode transition
const int   PWM_BITS        = 12;             // PWM resolution
const float PWM_MAG         = (float)(pow(2, PWM_BITS)-0); // PWM magnitude
#define MAG_CYCLES_PER_REV  50.f              // Electrical cycles per mechanical revolution

const uint32_t RATE_PWM_FREQ_HZ = uStepPerTurn*5; // FOR TESTING

// ISR timing
const uint32_t ISR_FREQ_HZ   = 25000UL;                 // ISR ticks per second
const uint32_t ISR_PERIOD_US = 1000000UL / ISR_FREQ_HZ; // dt in uSec
const float    ISR_DT        = 1.f/(float)ISR_FREQ_HZ;  // dt in sec

// Host rate steps/sec conversion to magnetic phase
const float rateToDeg  = 360.f/(float)uStepPerTurn;

// ADC
const int ADC_BITS = 12;

// --- Shared state (volatile for ISR/main) ---
volatile double demandAngle   = 0.0f;   // Demanded shaft angle (degrees) fine resolution due to ISR dt integration at low slew rates
volatile float  phaseAdv      = 0.0f;   // Phase advance
volatile float  pE            = 0.0f;   // Position error
volatile float  pEI           = 0.0f;   // Position error integration
volatile float  vE            = 0.0f;   // Velocity error
volatile float  vEI           = 0.0f;   // Velocity error integration
volatile float  vdE           = 0.0f;   // I2V D axis error
volatile float  vdEI          = 0.0f;   // I2V D axis error integration
volatile float  vqE           = 0.0f;   // I2V Q axis error
volatile float  vqEI          = 0.0f;   // I2V Q axis error integration
volatile float  dirSign       = 1;      // +1 CW, -1 CCW
volatile int8_t enableDrv     = HIGH;   // Drive enable
volatile int    releaseForUse = 0;      // Stepper released for normal drive
volatile float  freqMeasureHz = 0.f;    // Host step frequency

volatile int   isrCount = 0;

FreqMeasureMulti FreqMeasure1;           // Measure host step rate
FreqMeasureMulti FreqMeasure2;          // Measure encoder phase A rate

// Slew-limited output frequency for phase integration
volatile float slewFreqHz = 0.f;        // deg/sec
const float SLEW_RATE_HZ_PER_ISR = (float)uStepPerTurn * 5.f / (float)ISR_FREQ_HZ;

// Driver motor currents
const    float convCS   = (3.3f/(float)pow(2,ADC_BITS)) * (1.0f/1.1f); // CS scaling: 1.1V/A
volatile float Ia    = 0.f;  // Motor coil current A
volatile float Ib    = 0.f;  // Motor coil current B
volatile float Id    = 0.f;  // Rotor frame current - D axis
volatile float Iq    = 0.f;  // Rotor frame current - Q axis
volatile float IqDem = 0.f;  // Rotor frame demanded current - Q axis
volatile float IaPol = 1.f;  // Polarity of current A
volatile float IbPol = 1.f;  // Polarity of current B
volatile float Va    = 0.f;  // Duty cycle phase A
volatile float Vb    = 0.f;  // Duty cycle phase 
volatile float Vd    = 0.f;  // Duty cycle D 
volatile float Vq    = 0.f;  // Duty cycle Q
#define MAX_CURR       2.1f  // Max current limit DRV8824 (continuous)
#define SUP_VOLT      15.f   // Supply voltage
#define PHASE_ADV      5.f   // Phase advance per 360 deg/sec motor velocity
#define PHASE_ADV_MAX 30.f   // Maximum permissible phase advance
#define DIRECT_CURR   1.5f   // Direct mode target current (d axis)
// 15V supply, 4.8 mH, 1.13 ohms, 23HS30-2804D StepperOnline nema 23
#define V_KP          0.5f   // Proportional voltage gain (normalized)
#define V_KI          2.0f   // Integral voltage gain (normalized)
#define SERVO_KP      2.0f   // Position error gain
#define SERVO_KPI     1.0f   // Integrated position error gain
#define SERVO_KV    0.002f   // Velocity error gain
#define SERVO_KVI    0.01f   // Integrated velocity error gain

#define  ENC_REZ             2500    // Encoder counts per turn either phase
#define  ENC_QUAD_REZ   (ENC_REZ * 4)// Encoder quadrature counts per turn
Encoder motorEncoder(PIN_ENC_A, PIN_ENC_B);
volatile long  encCnt     = 0;    // Encoder quadrature counts
volatile int   encIndex   = -1;   // Encoder index detection count
volatile float encDeg     = 0.f;  // Encoder shaft angle in degrees
volatile float encOffset  = 0.f;  // Encoder offset (to magnetic cycle)
volatile float encVel     = 0.f;  // Encoder computed velocity from phase A period
volatile float encVel2    = 0.f;  // Encoder computed velocity from encoder counts
volatile float commNrm    = 0.f;  // Commutation angle (normalized)
volatile float commRad    = 0.f;  // Commutation angle (radians)
volatile float magRad     = 0.f;  // Resolved magnetic angle (radians)
volatile float slewRate   = 0.f;  // Motor slew rate deg/sec shaft angle
volatile float motorVel   = 0.f;  // Motor velocity
#define PH_CORR_CNT  (int)(ENC_QUAD_REZ/MAG_CYCLES_PER_REV) // Phase correction LUT elements
//#define PH_CORR_CNT  ENC_QUAD_REZ // Phase correction LUT elements
volatile float phCorrLUT[PH_CORR_CNT] = {0.f};  // Phase correction in direct mode trained LUT

#define limit0(A)     (((A)>1.f)   ? 1.f         : (((A)<-1.f)   ? -1.f        : (A)))
#define limit1(A,B)   (((A)<(-B))  ? (-B)        : (((A)> (B))   ?  (B)        : (A)))
#define limit2(A,B,C) (((A)<(B))   ? (B)         : (((A)> (C))   ?  (C)        : (A)))
#define limit180(A)   (((A)>180.f) ? ((A)-360.f) : (((A)<-180.f) ? ((A)+360.f) : (A)))
#define limit360(A)   (((A)>360.f) ? ((A)-360.f) : (((A)<0.f)    ? ((A)+360.f) : (A)))
#define limit360D(A)  (((A)>360.)  ? ((A)-360.)  : (((A)<0.)     ? ((A)+360.)  : (A)))
#define db(A,B)       (((A)>(B))?((A)-(B)):(((A)<-(B))?((A)+(B)):0))
#define hys(A,B,C)    (((A-B)>(C))?(A-C):(((A-B)<-C)?(A+C):B))
#define sign(A)       (((A) < 0)   ? -1   : 1)
#define signf(A)      (((A) < 0.f) ? -1.f : 1.f)

void DRV8874(float val, int PIN_PH, int PIN_EN)
{
  digitalWrite(PIN_PH, (val < 0.f) ? LOW : HIGH); // Reverse or Forward
  analogWrite(PIN_EN, (uint16_t)fabsf(val * PWM_MAG));
}

void encoderIndexISR() 
{
  motorEncoder.write(0); // Zero the encoder position on index pulse
  encCnt = 0;            // Zero the tracked position
  encIndex++;
}

// --- ISR: measurement + integration ---
void stepISR()
{
  digitalWriteFast(PIN_ISR_TOGGLE, HIGH);  // Debug
isrCount++;
if (isrCount > (int)ISR_FREQ_HZ) isrCount = 0;
  // Motor currents
  float angle = atan2f(Vb, Va);   // electrical angle of voltage vector
  IaPol = (cosf(angle) < 0.f) ? -1.f : 1.f;
  IbPol = (sinf(angle) < 0.f) ? -1.f : 1.f;

  Ia += (analogRead(PIN_CS_A) * convCS * IaPol - Ia) * 0.1f;
  Ib += (analogRead(PIN_CS_B) * convCS * IbPol - Ib) * 0.1f;

  // Encoder feedback, motor shaft position, magnetic angle
  encCnt = motorEncoder.read() % ENC_QUAD_REZ;
  if (encCnt < 0) encCnt += ENC_QUAD_REZ;
  encDeg  = (float)encCnt * (360.0f / (float)ENC_QUAD_REZ) - encOffset;
  encDeg = limit360(encDeg);
  float temp = encDeg * MAG_CYCLES_PER_REV * (1.f/360.f);
//commNrm = temp - (int)temp;  // Commutation magnetic cycle normalized
//if (commNrm < 0.f) commNrm += 1.f;
  commNrm = temp - floorf(temp);   // Always yields [0,1)
  commRad = commNrm * 2 * M_PI;
 
  // Encoder pulse period - velocity
  int fcnt = 0;
  uint32_t fsum = 0;
  static uint32_t lastEdgeTime = 0;
  while (FreqMeasure2.available())
  {
    fcnt++;
    fsum += FreqMeasure2.read();
    lastEdgeTime = micros();   // record time of last encoder edge
  }

  static float dirVel = 1.f;
  static long  encCntP = 0;

  if (encCntP != encCnt)
  {
    long diff = encCnt - encCntP;
    if      (diff >  (ENC_QUAD_REZ>>1)) diff -= ENC_QUAD_REZ;
    else if (diff < -(ENC_QUAD_REZ>>1)) diff += ENC_QUAD_REZ;
    dirVel =  (float)sign(diff);
    encCntP = encCnt;
  }

  static int encDtCnt = 0;

  encDtCnt++;
  if (encDtCnt > 100)
  {
    static long encCntP_ = 0;

    long diff = encCnt - encCntP_;
    if      (diff >  (ENC_QUAD_REZ>>1)) diff -= ENC_QUAD_REZ;
    else if (diff < -(ENC_QUAD_REZ>>1)) diff += ENC_QUAD_REZ;
    encVel2 =  diff * (360.f/ENC_QUAD_REZ) * ISR_FREQ_HZ / encDtCnt;
    encCntP_ = encCnt;
    encDtCnt = 0;
  }

  if (fcnt)
  {
    encVel = FreqMeasure2.countToFrequency(fsum/fcnt) * (360.f / ENC_REZ) * dirVel;
    if (fabsf(slewRate - (float)encVel) < (slewRate * .1f))
      motorVel += (encVel - motorVel) * 0.02f;
    else
      motorVel += (encVel2 - motorVel) * 0.02f;
  } 
  else if (micros() - lastEdgeTime > 100000) // 100 ms without edges
  { 
    motorVel = 0.f;
    encVel   = 0.f;
    encVel2  = 0.f;
  }
//// motorVel = motorVel * 0.98f + encVel * 0.02f;  maybe, AI suggssted

  // d-q currents
  Id =  cosf(commRad) * Ia + sinf(commRad) * Ib;
  Iq = -sinf(commRad) * Ia + cosf(commRad) * Ib;

  // Signal frequency (commanded rate and direction)
  if (releaseForUse)
  {
    // 1) Sample host step frequency, direction and enable
    enableDrv = digitalReadFast(PIN_HOST_ENABLE) ? HIGH : LOW;
    dirSign   = digitalReadFast(PIN_HOST_DIR   ) ? 1.f : -1.f;
   
    fcnt = 0;
    fsum = 0;
    while (FreqMeasure1.available())
    {
      fcnt++;
      fsum += FreqMeasure1.read();
    }

    if (fcnt) 
    {
      freqMeasureHz = (float)FreqMeasure1.countToFrequency(fsum/fcnt) * dirSign;
      freqMeasureHz = limit1(freqMeasureHz, maxFreq); // Limit to system capability
      lastHostStepTime = millis(); // Host is alive: record activity and clear timeout
      hostTimedOut = false;
    }
   
    if (freqMeasureHz > fMax) fMax = freqMeasureHz;
    if (freqMeasureHz < fMin) fMin = freqMeasureHz;
    fMax += (freqMeasureHz - fMax) * 0.0001;
    fMin += (freqMeasureHz - fMin) * 0.0001; 

    // --- Host timeout-aware target frequency ---
    if (hostTimedOut || (enableDrv == LOW)) freqMeasureHz = 0.0f; // Force a controlled ramp-down
   
    // Slew output frequency toward input frequency
    float freqDelta = freqMeasureHz - slewFreqHz;
    float deltaLim = 0;

    if (fabsf(slewFreqHz) < 500.f) 
      deltaLim = 40.f;
    else
      deltaLim = SLEW_RATE_HZ_PER_ISR;

    if      (freqDelta >  deltaLim) slewFreqHz += deltaLim;
    else if (freqDelta < -deltaLim) slewFreqHz -= deltaLim;
    else                            slewFreqHz  = freqMeasureHz;
  }  // releaseForUse

  slewRate     = slewFreqHz * rateToDeg;            // degrees/sec
  demandAngle += (double)slewRate * (double)ISR_DT; // deg/sec/iteration
  demandAngle  = limit360D(demandAngle);            // limit to single turn

  // 4) Motor servo and current control
  if ((fabsf(slewFreqHz) < 1.f) && releaseForUse)
  {
    // Motor has fully ramped down; now it's safe to brake
    pinMode(PIN_A_EN, OUTPUT);
    digitalWriteFast(PIN_A_EN, LOW);
    pinMode(PIN_B_EN, OUTPUT);
    digitalWriteFast(PIN_B_EN, LOW);
    pE  = 0.f;
    vE  = 0.f;
    pEI = 0.f;
    vEI = 0.f;
  }
  else
  {
    if (releaseForUse)   // Runtime
    {
      static bool inFOC = false;

      float absFreq = fabsf(slewFreqHz);
      
      // hysteresis about transFreqHz
      const float transHi = transFreqHz * 1.2f;
      const float transLo = transFreqHz * 0.8f;
      
      // FOC <> Direct transitions
      if ((absFreq > transHi) && !inFOC) 
      {
//      float Tdir = SERVO_KP * pE + SERVO_KPI * pEI;   // Direct torque
//      vE  = slewRate - motorVel;
//      vEI = (Tdir - SERVO_KV * vE) / SERVO_KVI;       // align torque
//      vEI = (Iq - SERVO_KV * vE) / SERVO_KVI;
        vEI  = (slewFreqHz > 0.f) ? abs(vEI)  : -abs(vEI);
        vdEI = (slewFreqHz > 0.f) ? abs(vdEI) : -abs(vdEI);
        vqEI = (slewFreqHz > 0.f) ? abs(vqEI) : -abs(vqEI);

        inFOC = true;
      }
      else if ((absFreq < transLo) && inFOC) 
      {
        demandAngle = (double)(encDeg + Iq/DIRECT_CURR);  // Align demand
        Vd    = .14f;
        vEI   = (slewFreqHz > 0.f) ? abs(vEI)  : -abs(vEI);
        inFOC = false;
      }
      
      if (inFOC)  // FOC & commutation mode
      {
//      pEI  = 0.f;
        vE   = slewRate - motorVel;
        vE  = limit1(vE,  500.f);
        vEI += vE * ISR_DT;
        vEI = limit1(vEI, 100.f);

        phaseAdv = motorVel * (PHASE_ADV/360.f);  // Degrees phase advance (motor vel is deg/sec)
        phaseAdv = limit1(phaseAdv, PHASE_ADV_MAX);
        magRad   = commRad + phaseAdv * (M_PI/180.f);

        IqDem  = (SERVO_KV * vE + SERVO_KVI * vEI);
        IqDem  = limit1(IqDem, 1.0f);   // MAX_CURR);

        // --- PI on Id (target = 0) ---
        vdE   = -Id;
        vdEI += V_KI * vdE * ISR_DT;
        vdEI  = limit0(vdEI);
        Vd    = V_KP * vdE + vdEI;
        Vd    = limit0(Vd);
        
        // --- PI on Iq ---
        vqE   = IqDem - Iq;
        vqEI += V_KI * vqE * ISR_DT;
        vqEI  = limit0(vqEI);
        Vq    = V_KP * vqE + vqEI;
        Vq    = limit0(Vq);
        
      }
      // Direct mode
      else
      {
        pE   = (float)demandAngle - encDeg;
        pE   = limit180(pE);
        pE   = limit1(pE, 1.f);
        pEI += pE * 0.0001f;
        pEI  = limit1(pEI, 2.f);
     
//      int   phCorrIndex = (int)(commNrm * (float)PH_CORR_CNT);
//      phCorrIndex = limit2(phCorrIndex, 0, (PH_CORR_CNT-1));
//      int phCorrIndex = limit2(encCnt, 0, (ENC_QUAD_REZ-1));  // To be safe!
        float phaseCorr = SERVO_KP * pE + SERVO_KPI * pEI;
//      phaseCorr = limit1(phaseCorr, 1.f);
//      float phase = ((float)demandAngle + phaseCorr + phCorrLUT[phCorrIndex] * signf(slewRate)) * MAG_CYCLES_PER_REV * (1.f/360.f);
        float phase = ((float)demandAngle + phaseCorr) * MAG_CYCLES_PER_REV * (1.f/360.f);
        phase = phase - floorf(phase); 
//      phase = fmodf(phase, 1.0f);
//      if (phase < 0.f) phase += 1.0f;
        magRad = phase * 2.f * M_PI;   // Convert to Radians
        // Training
//      if ((fabsf(slewRate - 360.f/240.f) < .5f) && 0) // Only aggregate when near sidereal
//      {
//        phCorrLUT[phCorrIndex] += phaseCorr * ISR_DT * 50.f;
//        phCorrLUT[phCorrIndex]  = limit1(phCorrLUT[phCorrIndex], 0.5f);
//      }
     
        Vd += (DIRECT_CURR - fabsf(Id)) * 0.0001f;
        Vq  = 0.f;
        Vd  = limit2(Vd, 0.f, 1.f);
        Vq  = limit2(Vq, 0.f, 1.f);
      }
    }
    // Initialization mode: index search & commutation offset (open loop)
    else
    {
      Vd = 0.15f;
      Vq = 0.f;
      float phase = (float)demandAngle * MAG_CYCLES_PER_REV * (1.f/360.f);
      magRad = (phase - (int)phase) *  2.f * M_PI;  // Convert to Radians
    }

    Va = cosf(magRad) * Vd - sinf(magRad) * Vq;
    Vb = sinf(magRad) * Vd + cosf(magRad) * Vq; 

    // DRV8874 PH/EN mode: set phase and PWM for each winding
    DRV8874(Va, PIN_A_PH, PIN_A_EN);    
    DRV8874(Vb, PIN_B_PH, PIN_B_EN);   
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
  Serial.println("Sinusoidal micro-stepper via dual DRV8874");

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

  // ISR
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

  // Setup frequency counter/measure
  FreqMeasure1.begin(PIN_HOST_RATE);
  FreqMeasure2.begin(PIN_ENC_RATE);

  // RS485 comm (Rx2/TX2)
  Serial2.begin(RS485_BAUD);
  Serial2.transmitterEnable(PIN_DE_RE);

  // start ISR timer
  stepTimer.begin(stepISR, ISR_PERIOD_US);
}

// Transmit with DE/RE control
void rs485Write(const uint8_t *data, size_t len) 
{
  Serial2.write(data, len);
  Serial2.flush();                 // Wait for TX to finish
}

// Non‑blocking receive
void rs485Poll() 
{
  while (Serial2.available()) 
  {
   // uint8_t b = Serial2.read();
   // integrate into your parser here
  }
}

void sTab(const char *label, float value, int width = 10, int decimals = 2)
{
  Serial.print(label);

  char buf[32];
  dtostrf(value, width, decimals, buf);
  Serial.print(buf);
}

void loop() 
{
  static int      operStep    = 0;
  static uint32_t magPoleTime = 0;
  static uint32_t lastDebugMs = 0;
  static uint32_t lastDebugMs2 = 0;

  switch (operStep)
  {
    case 0:
      Serial.println("Index search");
      releaseForUse = 0;
      enableDrv     = HIGH;
      dirSign       = 1.0f;
      operStep      = (encIndex >= 0) ? 1 : 2;
      slewFreqHz    = uStepPerTurn * .1;  // 10 seconds for complete revolution
      break;

    case 1:
      // On index - move away from it
      Serial.println("On index, moving off index");
      operStep++;
      break;

    case 2:
      // Initialization - find encoder index
      if (encIndex >= 0)        // Ensure index is found
      {
        slewFreqHz  = 0.f;
        magPoleTime = millis();
        Serial.println("Found index, moving to magnetic pole");
        operStep++;
      }
      break;

    case 3:
      // Force rotor to one pole
      demandAngle = 0.;
      if ((millis() - magPoleTime) > 2000)
      {
        Serial.print("Magnetic pole found at ");
        Serial.println(encDeg);
        encOffset     = encDeg;
        demandAngle   = 0.;
        releaseForUse = 1;
        operStep++;
      }
      break;

    case 4:
      // Send shaft position to host (no Rx expected)
      uint8_t data[10] = {0};
      short Enc = -(int)((float)(encCnt * 0x3fff) / ENC_QUAD_REZ);  // Convert to match Servo57D 14 bits
      data[0] = 0x10 | ((Enc>>12) & 0xf);  // MSB
      data[1] = 0x20 | ((Enc>> 8) & 0xf);
      data[2] = 0x30 | ((Enc>> 4) & 0xf);
      data[3] = 0x40 | ((Enc>> 0) & 0xf);  // LSB
      rs485Write((const uint8_t *)&data, 4);
      break;
  }

  if ((operStep >= 4) && (millis() - lastDebugMs >= 100) && 1)
  {
//  sTab(" isrCount=",      isrCount);
//  sTab(" enableDrv=",     enableDrv);
//  sTab(" dirSign=",       dirSign);
    sTab(" freqMeasureHz=", freqMeasureHz);
//  sTab(" fMax=",          fMax);
//  sTab(" fMin=",          fMin);
    sTab(" slewFreqHz=",    slewFreqHz);
//  sTab(" commRad=",       commRad);
//  sTab(" magRad=",        magRad);
//  sTab(" Va=",            Va);
//  sTab(" Vb=",            Vb);
//  sTab(" Vd=",            Vd);
//  sTab(" Vq=",            Vq);
//  sTab(" Ia=",            Ia);
//  sTab(" Ib=",            Ib);
//  sTab(" Id=",            Id);
//  sTab(" Iq=",            Iq);
//  sTab(" IqDem=",         IqDem);
//  sTab(" IaPol=",         IaPol);
//  sTab(" IbPol=",         IbPol);
    sTab(" slewRate=",      slewRate);
//  sTab(" encVel=",        encVel);
//  sTab(" encVel2=",       encVel2);
//  sTab(" motorVel=",      motorVel);
    sTab(" encCnt=",        encCnt);
    sTab(" demandAngle=",   demandAngle);
    sTab(" encDeg=",        encDeg);
//  sTab(" phaseAdv=",      phaseAdv);
    sTab(" pE=",            pE);
    sTab(" pEI=",           pEI);
//  sTab(" vE=",            vE);
//  sTab(" vEI=",           vEI);
//  sTab(" vqE=",           vqE);
//  sTab(" vqEI=",          vqEI);

    Serial.println();

    lastDebugMs = millis();
  }

  if ((operStep >= 4) && (millis() - lastDebugMs2 >= 10000) && 0)
  {
//  for (int i=0; i<PH_CORR_CNT; i++)
    for (int i=0; i<30; i++)
    {
      Serial.print(phCorrLUT[i]);
      Serial.print(" ");
    }
    Serial.println();
    lastDebugMs2 = millis();
  }

  // --- Host step loss detection ---
  if (!hostTimedOut && ((millis() - lastHostStepTime) > LOSS_TIMEOUT_MS)) hostTimedOut = true;

  delay(20);
}
