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
#include <SPI.h>

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
const int PIN_ENC_IDX      = 14;  // Encoder Index (Z)  [was 12 with disk encoder]
const int PIN_ENC_RATE     = 23;  // Measure encoder pulse rate (phase A) - pairs with 22
const int PIN_ENC_MOSI     = 11;  // Encoder comm (SPI0 - define here only to lock pin from being used)
const int PIN_ENC_MISO     = 12;  // Encoder comm (SPI0 - define here only to lock pin from being used)
const int PIN_ENC_SCK      = 13;  // Encoder comm (SPI0 - define here only to lock pin from being used)

const int PIN_HOST_ENABLE  = 20;  // Host motor enable DIP
const int PIN_HOST_DIR     = 21;  // Host motor direction DIP
const int PIN_HOST_RATE    = 22;  // Host requested rate (freq) - pairs with 23

const int PIN_RX2          = 7;   // RS485 receive comm with host
const int PIN_TX2          = 8;   // RS485 transmit comm with host
const int PIN_DE_RE        = 6;   // RS485 bus enable

const int PIN_ISR_TOGGLE   = 10;  // Debug - ISR active signal (monitor CPU usage by ISR)
const int PIN_RATE_PWM     = 9;   // Debug - local motor rate
const int PIN_LED          = 13;  // On-board LED (also SPI SCK)

const int PIN_ENC_CS       = 19;  // AS5047D chip select (CSn)

const uint32_t RS485_BAUD = 115200; // RS485 host comm baud rate

// Waveform / motor geometry
const int   BPWM_FREQ       = 25000;          // PWM frequency for DRV8874 driver
const uint32_t uStepPerTurn = 51200;          // Microsteps per revolution
const float maxFreq         = uStepPerTurn*10; // Maximum frequency (set according P/S & stepper specs)
const float transFreqHz     = uStepPerTurn/2; // Frequency for drive mode transition
const int   PWM_BITS        = 12;             // PWM resolution
const float PWM_MAG         = (float)(pow(2, PWM_BITS)-0); // PWM magnitude
#define MAG_CYCLES_PER_REV  50                // Electrical cycles per mechanical revolution
#define CAL_CNT MAG_CYCLES_PER_REV            // AS5074D calibration indexes

const uint32_t RATE_PWM_FREQ_HZ = uStepPerTurn*5; // Debug - FOR TESTING

// ISR timing
const uint32_t ISR_FREQ_HZ   = 25000UL;                 // ISR ticks per second
const uint32_t ISR_PERIOD_US = 1000000UL / ISR_FREQ_HZ; // dt in uSec
const float    ISR_DT        = 1.f/(float)ISR_FREQ_HZ;  // dt in sec

// Host rate steps/sec conversion to magnetic phase
const float rateToDeg  = 360.f/(float)uStepPerTurn;

// ADC
const int ADC_BITS = 12;

// AS5047D uses SPI MODE1, up to 10 MHz
SPISettings encSPI(10000000, MSBFIRST, SPI_MODE1);

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

FreqMeasureMulti FreqMeasure1;          // Measure host step rate
FreqMeasureMulti FreqMeasure2;          // Measure encoder phase A rate

// Slew-limited output frequency for phase integration
volatile float slewFreqHz = 0.f;        // deg/sec
const float SLEW_RATE_HZ_PER_ISR = (float)uStepPerTurn * 5.f / (float)ISR_FREQ_HZ;

// Driver motor currents
#define MAX_CURR       2.1f  // Max current limit DRV8824 (continuous)
#define SUP_VOLT      15.f   // Supply voltage
#define PHASE_ADV      5.f   // Phase advance per 360 deg/sec motor velocity
#define PHASE_ADV_MAX 30.f   // Maximum permissible phase advance
#define DIRECT_CURR   1.5f   // Direct mode target current (d axis)
// 15V supply, 4.8 mH, 1.13 ohms, 23HS30-2804D StepperOnline nema 23
// 15V supply, 3.0 mH, 1.00 ohms, 57BYGH633 Servo57D stepper Nema 23
#define V_KP          0.5f   // Proportional voltage gain (normalized)
#define V_KI          2.0f   // Integral voltage gain (normalized)
#define SERVO_KP      1.0f   // Position error gain (was 2 with external 00ppr encoder)
#define SERVO_KPI     0.5f   // Integrated position error gain
#define SERVO_KV    0.002f   // Velocity error gain
#define SERVO_KVI    0.01f   // Integrated velocity error gain
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
volatile float IdReq = DIRECT_CURR; // Rotor frame 0 D axis requested current

int UseExternalEncoder = 0;  // Use external encoder (ABI) otherwise AS5074D SPI magnetic position 14 bits
//#define  ENC_REZ           2500 // Encoder counts per turn either phase - glass external encoder
#define ENC_REZ              512  // Encoder counts per turn either phase - AS5074D ABI default boot up value (and max)
#define ENC_QUAD_REZ (ENC_REZ * 4)// Encoder quadrature counts per turn
#define MAG_ENC_REZ    (1 << 14)  // AS5074D SPI magnetic position resolution   
#define CAL_NONE               0  // AS5074D calibration not done
#define CAL_LEARN              1  // AS5074D calibration learn (sweeping)
#define CAL_DONE               2  // AS5074D calibration learn completed
#define CAL_VALID              3  // AS5074D calibration valid
Encoder motorEncoder(PIN_ENC_A, PIN_ENC_B);
volatile float  motDeg    = 0.f;  // Motor shaft position in degrees
volatile uint16_t magStat = 0;    // AS5074D angle error status bit
volatile uint16_t magRaw  = 0;    // AS5074D 14 bit position + error + parity
volatile uint16_t magDec  = 0;    // AS5074D 14 bit position
volatile float magDeg     = 0;    // AS5074D linearized degrees
volatile float magVel     = 0.f;  // AS5074D velocity in deg/sec
volatile int   magCalRun  = CAL_NONE; // AS5074D calibration run
volatile int   poleNum    = 0;    // AS5047D calibration run pole number
volatile bool  magBusy    = false;// AS5074D API busy
volatile long  encCnt     = 0;    // Encoder quadrature counts
volatile int   encIndex   = -1;   // Encoder index detection count
volatile float encDeg     = 0.f;  // Encoder shaft angle in degrees
volatile float encOffset  = 0.f;  // Encoder offset (to magnetic cycle)
volatile float encVel     = 0.f;  // Encoder computed velocity from phase A period
volatile float commNrm    = 0.f;  // Commutation angle (normalized)
volatile float commRad    = 0.f;  // Commutation angle (radians)
volatile float magRad     = 0.f;  // Resolved magnetic angle (radians)
volatile float slewRate   = 0.f;  // Motor slew rate deg/sec shaft angle
volatile float motorVel   = 0.f;  // Motor velocity

struct CalibPoint 
{
  uint16_t dec;  // raw AS5074D 14-bit angle (0–16383)
  float    deg;  // true mechanical angle (degrees)
  float    m;    // slope
  int      cal;  // calibration flag
};
CalibPoint magCal[CAL_CNT];

#define limit0(A)     (((A)>1.f)   ? 1.f         : (((A)<-1.f)   ? -1.f        : (A)))
#define limit1(A,B)   (((A)<(-B))  ? (-B)        : (((A)> (B))   ?  (B)        : (A)))
#define limit2(A,B,C) (((A)<(B))   ? (B)         : (((A)> (C))   ?  (C)        : (A)))
#define limit180(A)   (((A)>=180.f)? ((A)-360.f) : (((A)<-180.f) ? ((A)+360.f) : (A)))
#define limit360(A)   (((A)>=360.f)? ((A)-360.f) : (((A)<0.f)    ? ((A)+360.f) : (A)))
#define limit360D(A)  (((A)>=360.) ? ((A)-360.)  : (((A)<0.)     ? ((A)+360.)  : (A)))
#define db(A,B)       (((A)>(B))?((A)-(B)):(((A)<-(B))?((A)+(B)):0))
#define hys(A,B,C)    (((A-B)>(C))?(A-C):(((A-B)<-C)?(A+C):B))
#define sign(A)       (((A) < 0)   ? -1   : 1)
#define signf(A)      (((A) < 0.f) ? -1.f : 1.f)

// ---------------------------------------------------------------------------
// Command defines (with parity pre-applied)
// ---------------------------------------------------------------------------
#define CMD_ANGLECOM   0xFFFF // ANGLECOM (0xFFFF) → returns: [15]=parity, [14]=error, [13:0]=angle
#define CMD_ERRFL      0x4001 // read + parity - ERRFL register (0x0001)
#define CMD_STATUS     0x7FFD // read + parity - STATUS register (0x3FFD)
#define CMD_DIAG       0x7FFC // read + parity - DIAAGC register (0x3FFC)
#define AS5047_ANGLE14      0 // 14 bit angle only
#define AS5047_RAW          1 // raw 16‑bit frame (parity + error + angle)
#define AS5047_ERRFL        2 // error flags
#define AS5047_STATUS       3 // status register
#define AS5047_DIAG         4 // diagnostics (AGC + MAG + flags)

uint16_t AS5047_Read(uint16_t what)
{
  uint16_t raw;

  auto spi_xfer = [&](uint16_t cmd) -> uint16_t 
  {
    uint16_t res;
    digitalWrite(PIN_ENC_CS, LOW);
    SPI.beginTransaction(encSPI);
    res = SPI.transfer16(cmd);
    SPI.endTransaction();
    digitalWrite(PIN_ENC_CS, HIGH);
    return res;
  };

  switch (what)
  {
    case AS5047_ANGLE14:
      // Clean 14-bit angle (0–16383)
      raw = spi_xfer(CMD_ANGLECOM);
      return raw & 0x3FFF;

    case AS5047_RAW:
      // Full 16-bit frame:
      // bit15 = parity, bit14 = error, bits13:0 = angle
      return spi_xfer(CMD_ANGLECOM);

    case AS5047_ERRFL:
      // ERRFL bits:
      // bit0 = framing error
      // bit1 = invalid command
      // bit2 = parity error
      // bit3 = diagnostic error
      // bit4 = magnet too strong
      // bit5 = magnet too weak
      // bit6 = CORDIC overflow
      return spi_xfer(CMD_ERRFL);
  
    case AS5047_STATUS:
      // STATUS bits:
      // bit3 = MAGH (mag too strong)
      // bit4 = MAGL (mag too weak)
      // bit5 = COF (CORDIC overflow)
      // bit6 = OCF (offset compensation finished)
      // bit7 = AGC overflow
          return spi_xfer(CMD_STATUS);
    
    case AS5047_DIAG:
      // DIAAGC bits:
      // bits 7:0  = AGC value
      // bits 13:8 = MAG value
      // bit14     = LF  (loss of tracking)
      // bit15     = COF (CORDIC overflow)
      return spi_xfer(CMD_DIAG);
    
    default:
      return 0;
  }
}

void DRV8874(float val, int PIN_PH, int PIN_EN)
{
  digitalWrite(PIN_PH, (val < 0.f) ? LOW : HIGH); // Reverse or Forward
  analogWrite(PIN_EN, (uint16_t)fabsf(val * PWM_MAG));
}

void encoderIndexISR() 
{
  if (!releaseForUse)  // reset encoder per cycle causes tiny audible glitch
  {
    motorEncoder.write(0); // Zero the encoder position on index pulse
    encCnt = 0;            // Zero the tracked position
    encIndex++;
  }
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
  if (!magBusy)
  {
    magBusy = true;
    magRaw  = AS5047_Read(AS5047_RAW);
    magBusy = false;
    magStat = magRaw & 0x4000;     // Status (error)
    magDec  = magRaw & 0x3fff;     // 14 bit decimal position
    magDeg  = linearize1(magDec);  // LUT linearization 
//  magDeg  = linearize2(magDec);  // Fourrier coeff linearization
  }

  // --- SPI-derived velocity (deg/sec) ---
  static int   velCnt  = 0;
  static float prevDeg = 0.f;
  #define velCyc 20
  if (++velCnt >= velCyc)
  {
    float d = magDeg - prevDeg;
    d = limit180(d);  // handle wrap correctly
    float rawVel = d * ISR_FREQ_HZ * (1.f/velCyc);  // deg/sec

    // Low-pass filter (tune alpha 0.05–0.2)
    magVel += 0.1f * (rawVel - magVel);
    prevDeg = magDeg;
    velCnt = 0;
  }

  encCnt = (motorEncoder.read() % ENC_QUAD_REZ) * (UseExternalEncoder ? 1 : -1);
  if (encCnt < 0) encCnt += ENC_QUAD_REZ;
  encDeg = (float)encCnt * (360.0f / (float)ENC_QUAD_REZ) - encOffset;
  encDeg = limit360(encDeg);
  motDeg = UseExternalEncoder ? encDeg : magDeg;
  float temp = motDeg * MAG_CYCLES_PER_REV * (1.f/360.f);
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

  if (fcnt)
  {
    encVel = FreqMeasure2.countToFrequency(fsum/fcnt) * (360.f / ENC_REZ) * dirVel;
//  if (fabsf(slewRate - (float)encVel) < (slewRate * .1f))
//    motorVel += (encVel - motorVel) * 0.02f;
//    else
//    motorVel += (magVel - motorVel) * 0.02f;
  } 
  else if (micros() - lastEdgeTime > 100000) // 100 ms without edges
  { 
//  motorVel = 0.f;
    encVel   = 0.f;
  }
  // SPI position motor velocity (iso ABI period)
  motorVel += (magVel - motorVel) * 0.02f;

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

    // --- Host timeout-aware target frequency ---
    if (hostTimedOut || (enableDrv == LOW)) freqMeasureHz = 0.0f; // Force a controlled ramp-down
   
    // Slew output frequency toward input frequency
    float freqDelta = freqMeasureHz - slewFreqHz;
    float deltaLim = 0;

    if (fabsf(slewFreqHz) < 500.f) 
      deltaLim = 5000.f/(float)ISR_FREQ_HZ;
    else
      deltaLim = SLEW_RATE_HZ_PER_ISR;

    if      (freqDelta >  deltaLim) slewFreqHz += deltaLim;
    else if (freqDelta < -deltaLim) slewFreqHz -= deltaLim;
    else                            slewFreqHz  = freqMeasureHz;
  }  // releaseForUse

  if (slewFreqHz > fMax) fMax = slewFreqHz;
  if (slewFreqHz < fMin) fMin = slewFreqHz;
  fMax += (slewFreqHz - fMax) * 0.0001;
  fMin += (slewFreqHz - fMin) * 0.0001; 

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
        demandAngle = (double)(motDeg + Iq/IdReq);  // Align demand
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
        pE   = (float)demandAngle - motDeg;
        pE   = limit180(pE);
        pE   = limit1(pE, 1.f);
        pEI += pE * 0.0001f;
        pEI  = limit1(pEI, 2.f);
     
        float phaseCorr = SERVO_KP * pE + SERVO_KPI * pEI;
        float phase = ((float)demandAngle + phaseCorr) * MAG_CYCLES_PER_REV * (1.f/360.f);
        phase = phase - floorf(phase); 
        magRad = phase * 2.f * M_PI;   // Convert to Radians
     
        Vd += (IdReq - fabsf(Id)) * 0.0001f;
        Vq  = 0.f;
        Vd  = limit2(Vd, 0.f, 1.f);
        Vq  = limit2(Vq, 0.f, 1.f);
      }
    }
    // Initialization mode: index search & commutation offset (open loop) & linearization of magnetic encoder
    else
    {
      Vd = 0.15f;
      Vq = 0.f;
      float phase = (float)demandAngle * MAG_CYCLES_PER_REV * (1.f/360.f);
      magRad = (phase - (int)phase) *  2.f * M_PI;  // Convert to Radians

      if (magCalRun == CAL_LEARN)
      {
        static int k = 0;

        // Locate target table angle closest to current angle
        while ((k<CAL_CNT) && (magCal[k].deg < (float)demandAngle)) k++;
        if (k >= CAL_CNT) k = CAL_CNT-1;
        while ((k>0)       && (magCal[k].deg > (float)demandAngle)) k--;
        float diff = (float)demandAngle - magCal[k].deg;
        diff = limit180(diff);
        if ((magCal[k].cal != 1) && (diff > 0.f) && (diff < 0.01f))
        {
          magCal[k].cal = 1;
          magCal[k].dec = magDec;
          if (++poleNum >= CAL_CNT) magCalRun = CAL_DONE;
        }
      }
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

  // AS5074D magnetic encoder by SPI
  pinMode(PIN_ENC_CS, OUTPUT);
  digitalWrite(PIN_ENC_CS, HIGH);
  SPI.begin();

  // start ISR timer
  stepTimer.begin(stepISR, ISR_PERIOD_US);
}

// Transmit with DE/RE control
void rs485Write(const uint8_t *data, size_t len) 
{
  Serial2.write(data, len);
  Serial2.flush();                 // Wait for TX to finish
}

// Non blocking receive
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

// Fit e(dec) = a0 + Σ [ an*cos(n*phi) + bn*sin(n*phi) ], n=1..N
// where phi = 2*pi*dec/16384
// N = 3 is enough for AS5047D
#define N_HARM 3

float a0;
float a[N_HARM+1];
float b[N_HARM+1];
float kFit = 360.0f/(float)MAG_ENC_REZ;  // default
float offFit = 0.0f;

void fitFourier()
{
  const float ENC_MAX = (float)MAG_ENC_REZ;

  // 1) Linear fit: deg ≈ kFit * dec + offFit
  float Sx = 0, Sy = 0, Sxx = 0, Sxy = 0;
  for (int i = 0; i < CAL_CNT; i++)
  {
    float x = (float)magCal[i].dec;
    float y = magCal[i].deg;
    Sx  += x;
    Sy  += y;
    Sxx += x * x;
    Sxy += x * y;
  }

  float denom = (float)CAL_CNT * Sxx - Sx * Sx;
  kFit   = ((float)CAL_CNT * Sxy - Sx * Sy) / denom;
  offFit = (Sy - kFit * Sx) / (float)CAL_CNT;

  // 2) Build wrapped error vector
  float e[CAL_CNT];
  for (int i = 0; i < CAL_CNT; i++)
  {
    float ideal = kFit * (float)magCal[i].dec + offFit;
    float err   = magCal[i].deg - ideal;
    if (err > 180.f) err -= 360.f;
    if (err < -180.f) err += 360.f;
    e[i] = err;
  }

  // 3) a0
  float sum = 0;
  for (int i = 0; i < CAL_CNT; i++) sum += e[i];
  a0 = sum / (float)CAL_CNT;

  // 4) an, bn
  for (int n = 1; n <= N_HARM; n++)
  {
    float csum = 0, ssum = 0;
    for (int i = 0; i < CAL_CNT; i++)
    {
      float phi = TWO_PI * (float)magCal[i].dec / ENC_MAX;
      csum += e[i] * cosf(n * phi);
      ssum += e[i] * sinf(n * phi);
    }
    a[n] = (2.0f / (float)CAL_CNT) * csum;
    b[n] = (2.0f / (float)CAL_CNT) * ssum;
  }

  Serial.println("Fourier coefficients:");
  Serial.print("a0 = "); Serial.println(a0, 6);
  for (int n = 1; n <= N_HARM; n++)
  {
    Serial.print("a"); Serial.print(n);
    Serial.print(" = "); Serial.println(a[n], 6);
    Serial.print("b"); Serial.print(n);
    Serial.print(" = "); Serial.println(b[n], 6);
  }
}

void buildSegments()
{
  // --- 1. Sort rawTable[] by magnetic angle (ascending) ---
if (0)
{
  for (int i = 0; i < (CAL_CNT-1); i++) 
  {
    for (int j = i + 1; j < CAL_CNT; j++) 
    {
      if (magCal[j].dec < magCal[i].dec) 
      {
        CalibPoint tmp = magCal[i];
        magCal[i] = magCal[j];
        magCal[j] = tmp;
      }
    }
  }
}
  // --- 2. Build slope/intercept segments ---
  for (int i = 0; i < CAL_CNT; i++)
  {
    float x1, y1;
    float x0 = magCal[i].dec;
    float y0 = magCal[i].deg;

    if (i == (CAL_CNT-1))  // Wrap around
    {
      x1 = magCal[0].dec + MAG_ENC_REZ;
      y1 = magCal[0].deg + 360.f;
    }
    else
    {
      x1 = magCal[i+1].dec;
      y1 = magCal[i+1].deg;
    }

    float dx = x1 - x0;
    float dy = y1 - y0;

    if (dx != 0.f) magCal[i].m  = dy / dx; // slope
  }
  
  // Smooth wrap slope
  float m0  = magCal[0].m;
  float m49 = magCal[CAL_CNT-1].m;
  float mAvg = 0.5f * (m0 + m49);
  magCal[CAL_CNT-1].m = mAvg;


  magCalRun  = CAL_VALID;  // Allow interpolation

  Serial.println("Idx\t deg\t dec\t m");
  for (int i = 0; i < CAL_CNT; i++)
  {
    Serial.print(i);
    Serial.print("\t ");
    Serial.print(magCal[i].deg);
    Serial.print("\t ");
    Serial.print(magCal[i].dec);
    Serial.print("\t ");
    Serial.print(magCal[i].m, 6);
    Serial.println();
  }
}

float linearize1(uint16_t x)
{
  static int k = 0;
  uint32_t xw = x;
  if (magCalRun != CAL_VALID) return 0.f;

  if (x < magCal[0].dec) xw = x + MAG_ENC_REZ;

  while ((k<CAL_CNT) && (magCal[k].dec < xw)) k++;
  if (k >= CAL_CNT) k = CAL_CNT-1;
  while ((k>0)       && (magCal[k].dec > xw)) k--;
  float y = magCal[k].m * (xw - magCal[k].dec) + magCal[k].deg;
  y = limit360(y);

  return y;
}

float linearize2(uint16_t dec)
{
  const float ENC_MAX = (float)MAG_ENC_REZ;

  float x = (float)dec;

  // Base linear mapping (must match fitFourier)
  float ang = kFit * x + offFit;

  // Fourier correction
  float phi = TWO_PI * x / ENC_MAX;
  float corr = a0;
  for (int n = 1; n <= N_HARM; n++)
    corr += a[n] * cosf(n * phi) + b[n] * sinf(n * phi);

  ang += corr;
  ang  = limit360(ang);

  return ang;
}

void loop() 
{
  static int      operStep     = 0;
  static uint32_t magPoleTime  = 0;
  static uint32_t lastDebugMs  = 0;

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
        if (UseExternalEncoder)
          Serial.println(encDeg);
        else
          Serial.println(magDec);
        Serial.println("Proceeding to magnetic position sensor calibration");
        for (int i = 0; i < CAL_CNT; i++) magCal[i].deg = (float)i * 360.f/(float)CAL_CNT;
        encOffset  = encDeg;  // magDec
        magCalRun  = CAL_LEARN;
        slewFreqHz = (float)uStepPerTurn / 20.f; // 20 seconds per turn - calibration run
        operStep++;
      }
      break;

    case 4:
      // Sweep full cycle for mapping (non linearity LUT)
      if (magCalRun == CAL_DONE)
      {
        releaseForUse = 1;
        operStep      = 10;  // Initialization completed
        IdReq         = 0.5f * DIRECT_CURR;  // Lower direct current to reduce heating and noise
        buildSegments();     // Compute slope/intercepts for magnetic sensor linearization table
        fitFourier();        // Fit data to fourrier order
        Serial.println("Magnetic position sensor calibration completed");
        Serial.println("Driver released to host control");
      }
      break;

    case 10:
      // Send shaft position to host (no Rx expected)
      uint8_t data[10] = {0};
      short Enc = -(int)(motDeg * (float)0x3fff / 360.f);// Convert to match Servo57D 14 bits

      data[0] = 0x10 | ((Enc>>12) & 0xf);  // MSB
      data[1] = 0x20 | ((Enc>> 8) & 0xf);
      data[2] = 0x30 | ((Enc>> 4) & 0xf);
      data[3] = 0x40 | ((Enc>> 0) & 0xf);  // LSB
      rs485Write((const uint8_t *)&data, 4);
      break;
  }

  if (!magBusy && magStat)
  {
    magBusy = 1;
    //uint16_t err  = AS5047_Read(AS5047_ERRFL);
    //uint16_t stat = AS5047_Read(AS5047_STATUS);
    //uint16_t diag = AS5047_Read(AS5047_DIAG);
    magBusy = 0;
    // TODO: log errors
  }

  if ((operStep > 4) && (millis() - lastDebugMs >= 100) && 1)
  {
//  sTab(" isrCount=",      isrCount);
//  sTab(" enableDrv=",     enableDrv);
//  sTab(" dirSign=",       dirSign);
//  sTab(" freqMeasureHz=", freqMeasureHz);
//  sTab(" fMax=",          fMax);
//  sTab(" fMin=",          fMin);
//  sTab(" slewFreqHz=",    slewFreqHz);
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
//  sTab(" slewRate=",      slewRate);
    sTab(" encVel=",        encVel);
    sTab(" magVel=",        magVel);
    sTab(" motorVel=",      motorVel);
    sTab(" demandAngle=",   demandAngle);
//  sTab(" calTrg=",        calTrg);
    sTab(" magDeg=",        magDeg);
//  sTab(" magDec=",        magDec);
//  sTab(" encDeg=",        encDeg);
//  sTab(" encCnt=",        encCnt);
//  sTab(" magDec=",        magDec);
//  sTab(" magStat=",       magStat);
//  sTab(" phaseAdv=",      phaseAdv);
//  sTab(" pE=",            pE);
//  sTab(" pEI=",           pEI);
//  sTab(" vE=",            vE);
//  sTab(" vEI=",           vEI);
//  sTab(" vqE=",           vqE);
//  sTab(" vqEI=",          vqEI);

    Serial.println();

    lastDebugMs = millis();
  }

  // --- Host step loss detection ---
  if (!hostTimedOut && ((millis() - lastHostStepTime) > LOSS_TIMEOUT_MS)) hostTimedOut = true;

  delay(20);
}
