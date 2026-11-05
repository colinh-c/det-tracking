// =====================================================================
//  Precision Tracking Gimbal - motion firmware
//  Board : BigTreeTech SKR Pico v1.0  (RP2040 + 4x onboard TMC2209)
//  Core  : earlephilhower arduino-pico
//  Libs  : TMCStepper (teemuatlut)
//
//  Runs completely standalone. No host required for slew, hold, or
//  encoder sync. The Raspberry Pi only joins for camera tracking.
//
//  ARCHITECTURE (this is the important part - see Error Budget sheet)
//  -----------------------------------------------------------------
//  The AS5048A output encoder resolves 79 arcsec. The camera resolves
//  ~2.5 arcsec. The camera is ~32x finer. So the fine loop must NOT be
//  closed through the encoder, or you throw away 32x and limit-cycle at
//  the encoder LSB.
//
//    MODE_HOLD  : encoder closes the loop. Coarse. Deadband ~2 LSB.
//    MODE_SLEW  : trajectory generator runs open-loop on step counts,
//                 encoder only watches for lost steps.
//    MODE_TRACK : camera commands velocity + small position corrections
//                 DIRECTLY in fine microsteps. Encoder is a fault
//                 monitor only, never in the fine path.
//
//  Position is stored everywhere in FINE UNITS:
//      1 fine unit = FULL_STEP_DEG / 256 / GEAR_RATIO
//                  = 1.8 / 256 / 80 = 8.789e-5 deg = 0.316 arcsec
//  This is the commanded-resolution figure from the project plan.
// =====================================================================

#include <Arduino.h>
#include <TMCStepper.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"
#include "hardware/structs/sio.h"
#include "det_velocity_lease.h"
#include <errno.h>
#include <stdlib.h>

// Launch copy, 2026-11-03. See ../provenance.json for the preserved original.
// CAP/LVEL add an ISR-enforced automatic-motion lease. This file has not been
// flashed merely by compiling it. Legacy manual commands remain available
// only outside an active or expired leased session.

// ---------------------------------------------------------------------
//  MACHINE CONSTANTS  - change these to match your build
// ---------------------------------------------------------------------
static const float FULL_STEP_DEG = 1.8f;    // 200 step/rev NEMA 17
// MEASURED, not assumed. Camera-based calibration (host/gear_cal.py) gave
// 0.4829 actual deg per commanded deg against the old 80.0, i.e. an implied
// 165.7:1 with 3.3% scatter - within error of exactly 160:1. A strain-wave
// ratio is flexspline_teeth / tooth_difference, so a 160-tooth flexspline is
// 80:1 with a 2-tooth difference and 160:1 with 1 tooth. That single
// parameter differing between design and print gives exactly this factor of 2.
// Consequence: DEG_PER_FINE halves to 4.395e-5 deg (0.158 arcsec) and FINE
// mode's ceiling halves to ~1.76 deg/s. SLEW is unaffected (still clamps at
// MAX_VEL_DPS). Re-derive the workbook tiers from this.
static const float GEAR_RATIO    = 160.0f;  // strain-wave reduction, MEASURED
static const uint16_t USTEP_FINE = 256;     // fine pointing mode
static const uint16_t USTEP_SLEW = 16;      // slew mode (keeps step rate sane)

// derived
static const float DEG_PER_FINE  = FULL_STEP_DEG / 256.0f / GEAR_RATIO; // 8.789e-5
static const float FINE_PER_DEG  = 1.0f / DEG_PER_FINE;                 // 11377.8

// motion limits
static const float MAX_VEL_DPS   = 15.0f;   // project spec slew rate
// Acceleration is RUNTIME TUNABLE (ACC command). A stepper stalls when the
// commanded acceleration demands more torque than it can make; the symptom is
// a motor that buzzes and shakes without turning. Dropping accel is often a
// bigger win than raising current, and it costs no heat.
static volatile float MAX_ACC_DPS2 = 75.0f;   // reach full slew in 0.2 s
static const float HOME_VEL_DPS  = 3.0f;

// soft limits (degrees, relative to synced encoder zero)
static float AZ_MIN = -180.0f, AZ_MAX = 180.0f;
static float EL_MIN =  -10.0f, EL_MAX =  95.0f;

// TMC current. Torque margin is ~86x (see Drivetrain sheet), so run LOW.
// Less current -> less heat -> less thermal drift -> better error budget.
static const uint16_t RUN_CURRENT_MA  = 500;
static const uint16_t HOLD_MULT_PCT   = 50;

// ---------------------------------------------------------------------
//  SKR PICO v1.0 PIN MAP   (verified against Klipper generic config)
// ---------------------------------------------------------------------
// AZ uses the X driver slot, EL uses the Y driver slot.
#define AZ_STEP    11
#define AZ_DIR     10
#define AZ_EN      12
#define AZ_TMC_ADDR 0b00

#define EL_STEP     6
#define EL_DIR      5
#define EL_EN       7
#define EL_TMC_ADDR 0b10

// SKR Pico inverts DIR on the X/Y slots in stock configs.
#define AZ_DIR_INV  true
#define EL_DIR_INV  true

// TMC2209 shared UART bus (UART1)
#define TMC_TX      8
#define TMC_RX      9
static const float R_SENSE = 0.11f;   // SKR Pico sense resistors

// AS5048A encoders - BIT-BANGED SPI.
// Hardware SPI1 would need gpio26/27, which sit behind the thermistor
// RC filters on this board. Bit-banging on clean header pins avoids
// desoldering anything, and the RP2040 has speed to spare.
#define ENC_SCK    22   // probe header, "sensor" pin
#define ENC_MOSI   29   // probe header, "control" pin
#define ENC_MISO   24   // neopixel header, signal pin
#define ENC_CS_AZ  16   // filament-runout header
#define ENC_CS_EL  25   // Z endstop header

// limit switches
#define LIM_AZ      4   // X endstop header
#define LIM_EL      3   // Y endstop header

// ---------------------------------------------------------------------
//  TIMING
// ---------------------------------------------------------------------
static const uint32_t F_ISR    = 40000;   // step-generator tick, Hz
static const uint32_t PLAN_DIV = 40;      // planner runs at 1 kHz

// How quickly the axis executes a camera correction, in 1/s.
//
// This must be STIFF. The host measures error to the axis (via the camera)
// but applies its correction to the reference; anything between the two is
// lag the host cannot see, so it over-corrects and the two loops fight.
// At 200 the follower settles in ~5 ms, far inside one 100 fps camera frame,
// so the firmware is effectively a perfect position follower and the host's
// loop is the only one that matters.
//
// Stability: this runs in the 1 kHz planner, so the discrete gain is
// TRACK_KP * PLAN_DT = 0.2, well under 1. The acceleration limiter clamps
// the resulting demand, so a large step cannot slam the motor.
// It does not amplify camera noise: noise enters through trackRef at 100 Hz,
// an order of magnitude below this loop.
static const float TRACK_KP = 200.0f;

// Largest gap allowed between the tracking reference and the axis, degrees.
// Camera corrections are a few hundredths of a degree, so anything past this
// means something is wrong upstream. Bounds a runaway to a survivable speed.
static const float MAX_TRACK_ERR_DEG = 2.0f;

// ---------------------------------------------------------------------
//  DRIVERS
// ---------------------------------------------------------------------
TMC2209Stepper drvAZ(&Serial2, R_SENSE, AZ_TMC_ADDR);
TMC2209Stepper drvEL(&Serial2, R_SENSE, EL_TMC_ADDR);

// ---------------------------------------------------------------------
//  HOST LINK
//
//  The board answers on BOTH the USB CDC port and Serial1 (UART0, GPIO0/1),
//  which is what the SKR Pico's 5-pin "RPI" header brings out:
//      5V, 5V, GND, RXD(IO1), TXD(IO0)
//  Whichever the host is wired to, the protocol is identical. Serial2 is
//  UART1 on GPIO8/9 and belongs to the TMC2209 bus - do not confuse them.
//
//  NOTE: RP2040 GPIO is NOT 5V tolerant. IO0/IO1 must see 3.3V only.
// ---------------------------------------------------------------------
#define HOST_UART_TX 0
#define HOST_UART_RX 1

struct HostLink {
  template <class... A> void print(A... a)   { Serial.print(a...);   Serial1.print(a...); }
  template <class... A> void println(A... a) { Serial.println(a...); Serial1.println(a...); }
};
static HostLink Host;

// ---------------------------------------------------------------------
//  AXIS STATE
// ---------------------------------------------------------------------
enum Mode : uint8_t { MODE_IDLE, MODE_HOLD, MODE_SLEW, MODE_TRACK, MODE_FAULT };

struct Axis {
  // --- shared with ISR ---
  volatile int32_t  pos;          // current position, fine units
  volatile int32_t  target;       // commanded position, fine units
  volatile uint32_t accum;        // DDS phase accumulator
  volatile uint32_t incr;         // DDS increment (sets pulse rate)
  volatile int8_t   dir;          // +1 / -1
  volatile float    vel;          // current velocity, fine units/s (signed)
  volatile float    velCmd;       // feedforward velocity for MODE_TRACK
  volatile float    trackRef;     // tracking reference position, fine units
  volatile bool     posMode;      // true = seek target, false = tracking
  volatile int32_t  unitsPerPulse;// 256/ustep : 1 in fine mode, 16 in slew

  // --- main-thread only ---
  uint8_t  stepPin, dirPin, enPin, csPin, limPin;
  bool     dirInvert;
  uint16_t ustep;
  float    encDeg;                // last encoder reading
  float    encZeroDeg;            // encoder value that corresponds to pos 0
  int32_t  lostSteps;             // encoder-vs-commanded discrepancy, fine units
  bool     encOk;
  float    softMin, softMax;
  TMC2209Stepper *drv;
};

static Axis az, el;
static volatile Mode gMode = MODE_IDLE;
static volatile bool gEstop = false;
static DetVelocityLease gVelocityLease;

// Encoder watchdog enable. Bench bringup runs with no AS5048A wired, and a
// disconnected MISO idles LOW -> the frame reads 0x0000, which passes BOTH
// the parity check and the error flag and is accepted as a confident 0.000
// deg. encOk then goes true against nothing, lostSteps becomes -pos, and the
// axis faults the moment it moves past LOST_TRIP (~0.26 deg). Turning this
// off skips the encoder path entirely so the drivetrain can be exercised.
// Default stays ON: absent encoders must not be the silent default.
static volatile bool gEncEnabled = true;

// ---------------------------------------------------------------------
//  AS5048A  -  bit-banged SPI, mode 1 (CPOL=0, CPHA=1), MSB first
// ---------------------------------------------------------------------
static inline void encDelay() { delayMicroseconds(1); }   // ~500 kHz, very safe

static uint16_t as5048Transfer(uint8_t cs, uint16_t out) {
  uint16_t in = 0;
  digitalWrite(cs, LOW);
  encDelay();
  for (int i = 15; i >= 0; i--) {
    // CPHA=1: data changes on leading edge, sampled on trailing edge
    digitalWrite(ENC_SCK, HIGH);
    digitalWrite(ENC_MOSI, (out >> i) & 1);
    encDelay();
    digitalWrite(ENC_SCK, LOW);
    in = (in << 1) | (digitalRead(ENC_MISO) & 1);
    encDelay();
  }
  digitalWrite(cs, HIGH);
  encDelay();
  return in;
}

// even parity over bits 0..14
static bool parityOk(uint16_t v) {
  uint16_t x = v & 0x7FFF;
  x ^= x >> 8; x ^= x >> 4; x ^= x >> 2; x ^= x >> 1;
  return ((x & 1) == ((v >> 15) & 1));
}

// Returns angle in degrees 0..360, or -1.0 on error.
// 0xFFFF = read-angle command (reg 0x3FFF, R/W=1, even parity).
static float as5048ReadDeg(uint8_t cs) {
  as5048Transfer(cs, 0xFFFF);            // issue read
  uint16_t r = as5048Transfer(cs, 0xFFFF); // clock out the answer
  if (!parityOk(r))    return -1.0f;     // parity failure
  if (r & 0x4000)      return -1.0f;     // error flag set
  uint16_t raw = r & 0x3FFF;             // 14-bit
  return (float)raw * (360.0f / 16384.0f);
}

// Median of 3 rejects the occasional bad frame without adding lag.
static float as5048ReadDegFiltered(uint8_t cs) {
  float a = as5048ReadDeg(cs);
  float b = as5048ReadDeg(cs);
  float c = as5048ReadDeg(cs);
  if (a < 0 || b < 0 || c < 0) return -1.0f;
  float hi = max(a, max(b, c));
  float lo = min(a, min(b, c));
  return a + b + c - hi - lo;
}

// shortest signed difference between two angles, degrees
static float angDiff(float a, float b) {
  float d = a - b;
  while (d >  180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

// ---------------------------------------------------------------------
//  STEP GENERATOR  -  DDS at F_ISR
//
//  pulse_rate = incr * F_ISR / 2^32
//  incr       = pulse_rate * 2^32 / F_ISR
// ---------------------------------------------------------------------
static inline uint32_t rateToIncr(float pulsesPerSec) {
  if (pulsesPerSec < 0) pulsesPerSec = -pulsesPerSec;
  double v = (double)pulsesPerSec * 4294967296.0 / (double)F_ISR;
  if (v > 4294967295.0) v = 4294967295.0;
  return (uint32_t)v;
}

// fine units/s -> pulses/s, depends on current microstep mode
static inline float fineRateToPulseRate(const Axis &a, float finePerSec) {
  return finePerSec / (float)a.unitsPerPulse;
}

// The DDS can emit at most one pulse per ISR tick, so the achievable speed
// depends on which microstep mode you are in:
//    SLEW (1/16)  : 40000 pulses/s * 16 fine  = 56.2 deg/s   (spec is 15)
//    FINE (1/256) : 40000 pulses/s *  1 fine  =  3.5 deg/s
// Fine mode is for pointing, not slewing. Without this clamp a fast TRK
// command would silently saturate and the axis would quietly fall behind.
static inline float maxFineRate(const Axis &a) {
  return (float)F_ISR * (float)a.unitsPerPulse * 0.95f;   // 5% headroom
}

static struct repeating_timer gTimer;

static inline void axisStepISR(Axis &a, uint32_t &setMask) {
  if (a.incr == 0) return;
  uint32_t before = a.accum;
  a.accum += a.incr;
  if (a.accum < before) {                 // wrapped -> emit one pulse
    if (a.dir >= 0) a.pos += a.unitsPerPulse;
    else            a.pos -= a.unitsPerPulse;
    setMask |= (1u << a.stepPin);         // direct SIO, ~40x faster than digitalWrite
  }
}

// ---- planner, runs at 1 kHz inside the step ISR -----------------------
static const float PLAN_DT = 1.0f / (float)(F_ISR / PLAN_DIV);

static void axisPlan(Axis &a) {
  const float accFine = MAX_ACC_DPS2 * FINE_PER_DEG;   // re-read each tick
  // clamp to whichever is lower: the machine limit, or what the step
  // generator can physically emit in the current microstep mode
  const float maxFine = fminf(MAX_VEL_DPS * FINE_PER_DEG, maxFineRate(a));
  const float dv      = accFine * PLAN_DT;
  const float posF    = (float)a.pos;
  const float limLo   = a.softMin * FINE_PER_DEG;
  const float limHi   = a.softMax * FINE_PER_DEG;

  float v = a.vel;

  if (gEstop || gVelocityLease.tripped) {
    // decelerate hard, do not just cut the step train
    float d = gEstop ? dv * 3.0f : dv;
    v = detLeaseBrake(v, d);
    // The lease brake ignores stale target/reference error. Once stopped,
    // hold the actual counted position, without snapping or changing zero.
    if (gVelocityLease.tripped && v == 0.0f) {
      a.target = a.pos;
      a.trackRef = (float)a.pos;
      a.velCmd = 0.0f;
      a.posMode = true;
    }
  }
  else {
    // Soft limits are enforced THROUGH the velocity profile, not as a
    // separate brake rule. A separate rule fights the trapezoid: the
    // profile adds dv while the brake subtracts dv, they cancel, and the
    // axis creeps past the fence at full speed. Clamping the target and
    // applying a sqrt(2*a*d) speed ceiling makes deceleration a property
    // of the profile itself, so the two can never disagree.
    float tgt = (float)a.target;
    if (tgt > limHi) tgt = limHi;
    if (tgt < limLo) tgt = limLo;

    float vWant;
    if (a.posMode) {
      float err  = tgt - posF;
      float dirf = (err >= 0) ? 1.0f : -1.0f;
      vWant = dirf * fminf(maxFine, sqrtf(2.0f * accFine * fabsf(err)));
    } else {
      // ---- camera tracking ----
      // The reference advances by itself at the feedforward rate, and the
      // camera's corrections nudge that reference. The axis then servos
      // onto it. Feeding a correction straight into position instead would
      // move the bookkeeping without moving the motor, and the host would
      // keep re-sending a correction that never gets executed - positive
      // feedback, and the axis runs away.
      a.trackRef += a.velCmd * PLAN_DT;
      if (a.trackRef > limHi) a.trackRef = limHi;
      if (a.trackRef < limLo) a.trackRef = limLo;

      // Hard bound on how far the reference may sit from the axis. The
      // follower commands speed proportional to that gap, so a stale or
      // bogus reference - a host bug, a garbled packet, entering tracking
      // without seeding - would otherwise slam the axis to full speed in
      // whichever direction the stale value happens to lie. This is a
      // physical machine; make that unreachable rather than unlikely.
      const float maxGap = MAX_TRACK_ERR_DEG * FINE_PER_DEG;
      if (a.trackRef > posF + maxGap) a.trackRef = posF + maxGap;
      if (a.trackRef < posF - maxGap) a.trackRef = posF - maxGap;

      // Follower: proportional, but never demanding more speed than the
      // acceleration limit can bleed off again over the remaining error.
      // A plain P term against a hard accel limit overshoots and then
      // oscillates as soon as TRACK_KP * err exceeds sqrt(2*a*err), i.e.
      // above err = 2*accel/TRACK_KP^2. This ceiling is the same
      // stopping-distance rule the soft limits use, and it makes the
      // follower stable at any gain.
      float err     = a.trackRef - posF;
      float vFollow = TRACK_KP * err;
      float vCap    = sqrtf(2.0f * accFine * fabsf(err));
      if (vFollow >  vCap) vFollow =  vCap;
      if (vFollow < -vCap) vFollow = -vCap;

      vWant = a.velCmd + vFollow;              // feedforward + follower
      if (vWant >  maxFine) vWant =  maxFine;
      if (vWant < -maxFine) vWant = -maxFine;
    }

    // Never travel faster than you can still stop before the fence.
    // Two margins: one planner tick of travel (detection latency) and one
    // step pulse (you cannot land finer than a single pulse). Without the
    // second, the axis settles one pulse past the limit - harmless, but a
    // limit that is sometimes crossed is not a limit.
    float look = fabsf(v) * PLAN_DT + (float)a.unitsPerPulse;
    float dHi  = limHi - posF - look; if (dHi < 0) dHi = 0;
    float dLo  = posF - limLo - look; if (dLo < 0) dLo = 0;
    if (vWant > 0) vWant = fminf(vWant,  sqrtf(2.0f * accFine * dHi));
    if (vWant < 0) vWant = fmaxf(vWant, -sqrtf(2.0f * accFine * dLo));

    if      (vWant > v + dv) v += dv;
    else if (vWant < v - dv) v -= dv;
    else                     v  = vWant;

    // snap onto the target inside one planner tick of travel
    if (a.posMode &&
        fabsf(tgt - posF) <= fabsf(v) * PLAN_DT + 1.0f &&
        fabsf(v) < dv * 2.0f) {
      a.pos = (int32_t)tgt;
      v = 0.0f;
    }
  }

  // hard backstop in case anything above let a step through
  if (posF <= limLo && v < 0) v = 0.0f;
  if (posF >= limHi && v > 0) v = 0.0f;

  a.vel = v;
  a.dir = (v >= 0) ? 1 : -1;
  digitalWrite(a.dirPin, a.dirInvert ? (a.dir > 0 ? LOW : HIGH)
                                     : (a.dir > 0 ? HIGH : LOW));
  a.incr = rateToIncr(fineRateToPulseRate(a, v));
}

static bool stepTimerCB(struct repeating_timer *) {
  static uint32_t planCount = 0;

  uint32_t setMask = 0;
  axisStepISR(az, setMask);
  axisStepISR(el, setMask);
  if (setMask) {
    sio_hw->gpio_set = setMask;        // both axes rise together
    busy_wait_us_32(2);                // TMC2209 needs >100 ns; 2 us is safe
    sio_hw->gpio_clr = setMask;
  }

  if (++planCount >= PLAN_DIV) {
    planCount = 0;
    if (gVelocityLease.tick(time_us_32())) {
      az.velCmd = 0.0f;
      el.velCmd = 0.0f;
    }
    axisPlan(az);
    axisPlan(el);
    if (gVelocityLease.tripped && az.vel == 0.0f && el.vel == 0.0f && !gEstop)
      gMode = MODE_HOLD;
  }
  return true;
}

// ---------------------------------------------------------------------
//  MICROSTEP MODE SWITCHING
//  Position bookkeeping stays in fine units, so switching is safe as long
//  as we round pos to a whole pulse of the new mode.
// ---------------------------------------------------------------------
static void setUstep(Axis &a, uint16_t us) {
  if (a.ustep == us) return;
  noInterrupts();
  a.incr = 0; a.vel = 0;                       // stop cleanly first
  interrupts();
  delay(2);

  a.drv->microsteps(us);
  a.ustep = us;
  int32_t upp = 256 / (int32_t)us;

  noInterrupts();
  // round current position onto the coarser grid so pulses stay aligned
  a.pos = (a.pos / upp) * upp;
  a.unitsPerPulse = upp;
  a.accum = 0;
  interrupts();
}

// ---------------------------------------------------------------------
//  ENCODER SYNC
//  With an ABSOLUTE encoder you do not need to home at power-on. Read the
//  encoder, and you already know where you are. Limit switches exist for
//  safety and for discovering mechanical travel, not for finding zero.
// ---------------------------------------------------------------------
static bool axisSync(Axis &a) {
  float d = as5048ReadDegFiltered(a.csPin);
  if (d < 0) { a.encOk = false; return false; }
  a.encOk  = true;
  a.encDeg = d;
  float rel = angDiff(d, a.encZeroDeg);
  noInterrupts();
  a.pos      = (int32_t)lroundf(rel * FINE_PER_DEG);
  a.target   = a.pos;
  a.trackRef = (float)a.pos;      // else the next TRK jumps by the sync delta
  interrupts();
  return true;
}

static void axisReadEnc(Axis &a) {
  float d = as5048ReadDegFiltered(a.csPin);
  if (d < 0) { a.encOk = false; return; }
  a.encOk  = true;
  a.encDeg = d;
  float encRelDeg = angDiff(d, a.encZeroDeg);
  int32_t encFine = (int32_t)lroundf(encRelDeg * FINE_PER_DEG);
  a.lostSteps = encFine - a.pos;
}

// ---------------------------------------------------------------------
//  SETUP
// ---------------------------------------------------------------------
static void axisInit(Axis &a, uint8_t stepPin, uint8_t dirPin, uint8_t enPin,
                     uint8_t csPin, uint8_t limPin, bool dirInv,
                     float sMin, float sMax, TMC2209Stepper *drv) {
  a.stepPin = stepPin; a.dirPin = dirPin; a.enPin = enPin;
  a.csPin   = csPin;   a.limPin = limPin; a.dirInvert = dirInv;
  a.softMin = sMin;    a.softMax = sMax;  a.drv = drv;
  a.pos = a.target = 0; a.accum = 0; a.incr = 0; a.dir = 1;
  a.vel = 0; a.velCmd = 0; a.trackRef = 0; a.posMode = true;
  a.ustep = USTEP_SLEW; a.unitsPerPulse = 256 / USTEP_SLEW;
  a.encZeroDeg = 0; a.lostSteps = 0; a.encOk = false;

  pinMode(stepPin, OUTPUT); digitalWrite(stepPin, LOW);
  pinMode(dirPin,  OUTPUT); digitalWrite(dirPin,  LOW);
  pinMode(enPin,   OUTPUT); digitalWrite(enPin,   HIGH);  // active low: disabled
  pinMode(csPin,   OUTPUT); digitalWrite(csPin,   HIGH);
  pinMode(limPin,  INPUT_PULLUP);
}

static void tmcInit(TMC2209Stepper &d) {
  d.begin();
  d.toff(5);
  d.rms_current(RUN_CURRENT_MA, (float)HOLD_MULT_PCT / 100.0f);
  d.blank_time(24);
  // CRITICAL: without this, MRES comes from the MS1/MS2 pins, which this
  // board uses for UART addressing. microsteps() would silently do nothing.
  d.mstep_reg_select(true);
  d.microsteps(USTEP_SLEW);
  d.pwm_autoscale(true);
  // stealthChop is quiet but gives up a LOT of torque, which is the wrong
  // trade against a stiff or binding drivetrain. Runtime switchable (SPRD).
  d.en_spreadCycle(false);   // stealthChop: smoother and quieter at low speed
  d.TPWMTHRS(300);           // hand over to spreadCycle once moving fast
  d.iholddelay(6);
  d.TPOWERDOWN(128);
}

void setup() {
  Serial.begin(115200);          // USB CDC to the Pi
  Serial1.setTX(HOST_UART_TX);   // GPIO0 -> SKR Pico "RPI" header TXD
  Serial1.setRX(HOST_UART_RX);   // GPIO1 -> SKR Pico "RPI" header RXD
  Serial1.begin(115200);         // same protocol, second wire

  pinMode(ENC_SCK,  OUTPUT); digitalWrite(ENC_SCK,  LOW);
  pinMode(ENC_MOSI, OUTPUT); digitalWrite(ENC_MOSI, LOW);
  pinMode(ENC_MISO, INPUT);

  axisInit(az, AZ_STEP, AZ_DIR, AZ_EN, ENC_CS_AZ, LIM_AZ, AZ_DIR_INV,
           AZ_MIN, AZ_MAX, &drvAZ);
  axisInit(el, EL_STEP, EL_DIR, EL_EN, ENC_CS_EL, LIM_EL, EL_DIR_INV,
           EL_MIN, EL_MAX, &drvEL);

  Serial2.setTX(TMC_TX);
  Serial2.setRX(TMC_RX);
  Serial2.begin(115200);
  delay(200);

  tmcInit(drvAZ);
  tmcInit(drvEL);

  // enable drivers (active low)
  digitalWrite(AZ_EN, LOW);
  digitalWrite(EL_EN, LOW);

  bool syncAz = axisSync(az);
  bool syncEl = axisSync(el);
  gMode = MODE_HOLD;

  add_repeating_timer_us(-(int64_t)(1000000 / F_ISR), stepTimerCB, NULL, &gTimer);

  Host.println(F("# gimbal ready"));
  Host.print(F("# deg/fine=")); Host.println(DEG_PER_FINE, 8);
  Host.print(F("# tmc az=")); Host.print(drvAZ.test_connection() == 0 ? "OK" : "FAIL");
  Host.print(F(" el="));       Host.println(drvEL.test_connection() == 0 ? "OK" : "FAIL");
  Host.print(F("# sync az=")); Host.print(syncAz ? "OK" : "FAIL");
  Host.print(F(" el="));       Host.println(syncEl ? "OK" : "FAIL");
  if (!syncAz || !syncEl) Host.println(F("# encoders did not answer - 'ENC 0' to bench without them"));
}

// ---------------------------------------------------------------------
//  SERIAL COMMAND PROTOCOL   (ASCII, newline terminated)
//
//   PING                       -> OK PONG
//   CAP                        -> OK CAP VEL_LEASE_V1
//   LSTAT                      -> OK LSTAT <active:0|1> <tripped:0|1> <tick_us>
//   LVEL <seq> <ttl_ms> <ref_us> <vAz> <vEl>
//                              -> OK LVEL <seq> <ttl_ms> <ref_us>
//                              atomic leased velocity, seq uint32 increasing
//                              until STOP, ttl_ms 1..500. Expiry latches braking
//                              in the timer ISR, not in USB command handling.
//   STAT                       -> OK <azDeg> <elDeg> <azEnc> <elEnc>
//                                    <azLost> <elLost> <mode> <flags>
//   SYNC                       re-read absolute encoders, adopt as truth
//   ZERO                       define current position as 0,0
//   MODE SLEW|FINE             set microstep mode
//   MOVA <azDeg> <elDeg>       absolute move, trapezoidal
//   MOVR <dAz>   <dEl>         relative move
//   NUDGE <nAz>  <nEl>         relative move in FINE UNITS (integer)
//   TRK <vAz> <vEl> <cAz> <cEl>  velocity feedforward + position correction
//                                (deg/s, deg) - the camera tracking command
//   VEL <vAz> <vEl>            pure velocity mode, deg/s
//   STOP                       decelerate to rest, return to HOLD
//   EST / CLR                  assert / clear emergency stop
//   ENC <0|1>                  disable/enable encoder watchdog (bench bringup)
//   DIAG                       dump TMC driver registers - use this when a
//                              motor has no holding torque at all
//   DSTAT                      one DRV_STATUS + enable pin per axis, OK DSTAT
//   ACC <deg/s2>               set acceleration limit (1..500) - lower this
//                              first if a motor buzzes instead of turning
//   SPRD <0|1>                 spreadCycle on (1) = more torque, louder;
//                              stealthChop (0) = quiet, noticeably weaker
//   CUR <mA>                   set run current
//   LIM <azMin> <azMax> <elMin> <elMax>
// ---------------------------------------------------------------------
static char lineBuf[128];
static uint8_t lineLen = 0;
static char lineBuf1[128];      // Serial1 needs its own; the two streams
static uint8_t lineLen1 = 0;    // must not interleave into one buffer

static void handleLine(char *s);          // defined below

static void pumpStream(Stream &s, char *buf, uint8_t &len) {
  while (s.available()) {
    char c = s.read();
    if (c == '\n' || c == '\r') {
      if (len) { buf[len] = 0; handleLine(buf); len = 0; }
    } else if (len < 127) {
      buf[len++] = c;
    }
  }
}

static void setPosTarget(Axis &a, float deg) {
  float d = constrain(deg, a.softMin, a.softMax);
  noInterrupts();
  a.posMode = true;
  a.target  = (int32_t)lroundf(d * FINE_PER_DEG);
  interrupts();
}

// Entering tracking must seed the reference from where the axis actually is,
// otherwise the first TRK jumps the axis by however far trackRef is stale.
// Call with interrupts already disabled.
static inline void enterTrack(Axis &a) {
  if (a.posMode) {
    a.trackRef = (float)a.pos;
    a.posMode  = false;
  }
}

// Leaving tracking: hold wherever we ended up.
static inline void exitTrack(Axis &a) {
  a.posMode = true;
  a.target  = a.pos;
  a.velCmd  = 0.0f;
}

static bool detParseU32(const char *text, uint32_t &value) {
  if (!text || !*text) return false;
  for (const char *p=text; *p; ++p) if (*p<'0' || *p>'9') return false;
  errno = 0;
  char *end = nullptr;
  unsigned long parsed = strtoul(text, &end, 10);
  if (errno || !end || *end || parsed > UINT32_MAX) return false;
  value = (uint32_t)parsed;
  return true;
}

static bool detParseVelocity(const char *text, float &value) {
  if (!text || !*text) return false;
  errno = 0;
  char *end = nullptr;
  value = strtof(text, &end);
  return !errno && end && !*end && isfinite(value) && fabsf(value) <= MAX_VEL_DPS;
}

static void handleLine(char *s) {
  while (*s == ' ') s++;
  char *cmd = strtok(s, " ");
  if (!cmd) return;

  if (!strcasecmp(cmd, "CAP")) {
    Host.println(F("OK CAP VEL_LEASE_V1"));
    return;
  }

  if (!strcasecmp(cmd, "LSTAT")) {
    noInterrupts();
    bool active = gVelocityLease.active, tripped = gVelocityLease.tripped;
    uint32_t tick = time_us_32();
    interrupts();
    Host.print(F("OK LSTAT ")); Host.print(active ? 1 : 0);
    Host.print(' '); Host.print(tripped ? 1 : 0);
    Host.print(' '); Host.println(tick);
    return;
  }

  if (!strcasecmp(cmd, "LVEL")) {
    char *p[5];
    for (int i=0; i<5; ++i) {
      p[i] = strtok(NULL, " ");
      if (!p[i]) { Host.println(F("ERR ARG")); return; }
    }
    uint32_t sequence, ttl, reference;
    float vA, vE;
    if (strtok(NULL, " ") || !detParseU32(p[0], sequence) || !detParseU32(p[1], ttl) ||
        !detParseU32(p[2], reference) || !detParseVelocity(p[3], vA) || !detParseVelocity(p[4], vE) ||
        !isfinite(MAX_ACC_DPS2) || MAX_ACC_DPS2 <= 0.0f) {
      Host.println(F("ERR ARG")); return;
    }
    noInterrupts();
    bool accepted = !gEstop && gVelocityLease.accept(sequence, ttl, reference, time_us_32());
    if (accepted) {
      enterTrack(az); enterTrack(el);
      az.velCmd = vA * FINE_PER_DEG;
      el.velCmd = vE * FINE_PER_DEG;
      gMode = MODE_TRACK;
    }
    interrupts();
    if (!accepted) { Host.println(F("ERR LEASE")); return; }
    Host.print(F("OK LVEL ")); Host.print(sequence); Host.print(' '); Host.print(ttl);
    Host.print(' '); Host.println(reference);
    return;
  }

  // Monitoring and Stop remain available while leased. A legacy command on
  // either UART cannot bypass an active lease or its latched expiry.
  if ((gVelocityLease.active || gVelocityLease.tripped) &&
      strcasecmp(cmd, "PING") && strcasecmp(cmd, "STAT") && strcasecmp(cmd, "DIAG") &&
      strcasecmp(cmd, "DSTAT") &&
      strcasecmp(cmd, "STOP") && strcasecmp(cmd, "EST")) {
    Host.println(F("ERR LEASE")); return;
  }

  if (!strcasecmp(cmd, "PING")) { Host.println(F("OK PONG")); return; }

  if (!strcasecmp(cmd, "STAT")) {
    Host.print(F("OK "));
    Host.print((float)az.pos * DEG_PER_FINE, 5); Host.print(' ');
    Host.print((float)el.pos * DEG_PER_FINE, 5); Host.print(' ');
    Host.print(az.encDeg, 4); Host.print(' ');
    Host.print(el.encDeg, 4); Host.print(' ');
    Host.print(az.lostSteps); Host.print(' ');
    Host.print(el.lostSteps); Host.print(' ');
    Host.print((int)gMode); Host.print(' ');
    Host.print((az.encOk ? 1 : 0) | (el.encOk ? 2 : 0) | (gEstop ? 4 : 0)
                 | (gEncEnabled ? 8 : 0));
    Host.println();
    return;
  }

  if (!strcasecmp(cmd, "SYNC")) {
    if (!gEncEnabled) { Host.println(F("ERR ENCOFF")); return; }
    bool a = axisSync(az), b = axisSync(el);
    Host.println((a && b) ? F("OK SYNC") : F("ERR ENC"));
    return;
  }

  if (!strcasecmp(cmd, "ZERO")) {
    az.encZeroDeg = az.encDeg; el.encZeroDeg = el.encDeg;
    axisSync(az); axisSync(el);
    Host.println(F("OK ZERO"));
    return;
  }

  if (!strcasecmp(cmd, "MODE")) {
    char *m = strtok(NULL, " ");
    if (!m) { Host.println(F("ERR ARG")); return; }
    uint16_t us = (!strcasecmp(m, "FINE")) ? USTEP_FINE : USTEP_SLEW;
    setUstep(az, us); setUstep(el, us);
    // report the speed ceiling for this mode so the host never has to guess
    float vmax = fminf(MAX_VEL_DPS, maxFineRate(az) * DEG_PER_FINE);
    Host.print(F("OK MODE ")); Host.print(us);
    Host.print(F(" VMAX ")); Host.println(vmax, 3);
    return;
  }

  if (!strcasecmp(cmd, "MOVA") || !strcasecmp(cmd, "MOVR")) {
    char *p1 = strtok(NULL, " "), *p2 = strtok(NULL, " ");
    if (!p1 || !p2) { Host.println(F("ERR ARG")); return; }
    float a = atof(p1), b = atof(p2);
    if (!strcasecmp(cmd, "MOVR")) {
      a += (float)az.pos * DEG_PER_FINE;
      b += (float)el.pos * DEG_PER_FINE;
    }
    setPosTarget(az, a); setPosTarget(el, b);
    gMode = MODE_SLEW;
    Host.println(F("OK MOV"));
    return;
  }

  if (!strcasecmp(cmd, "NUDGE")) {
    char *p1 = strtok(NULL, " "), *p2 = strtok(NULL, " ");
    if (!p1 || !p2) { Host.println(F("ERR ARG")); return; }
    noInterrupts();
    az.posMode = true; az.target = az.pos + atol(p1);
    el.posMode = true; el.target = el.pos + atol(p2);
    interrupts();
    Host.println(F("OK NUDGE"));
    return;
  }

  // The tracking command. Velocity feedforward is what takes Tier C from
  // 73 arcsec down to 12 - see the Trades sheet. Send this every frame.
  if (!strcasecmp(cmd, "TRK")) {
    char *p[4];
    for (int i = 0; i < 4; i++) { p[i] = strtok(NULL, " "); if (!p[i]) { Host.println(F("ERR ARG")); return; } }
    float vA = atof(p[0]), vE = atof(p[1]);
    float cA = atof(p[2]), cE = atof(p[3]);
    noInterrupts();
    enterTrack(az); enterTrack(el);
    az.velCmd = vA * FINE_PER_DEG;
    el.velCmd = vE * FINE_PER_DEG;
    // the correction shifts the reference the axis is chasing, which is
    // what actually turns it into motion
    az.trackRef += cA * FINE_PER_DEG;
    el.trackRef += cE * FINE_PER_DEG;
    int32_t pA = az.pos, pE = el.pos;
    interrupts();
    gMode = MODE_TRACK;
    // Reply with actual position. The host needs a MEASURED gimbal angle to
    // build the inertial-frame signal it estimates target velocity from.
    // If it dead-reckons that angle from its own velocity command instead,
    // the estimate feeds back on itself and the loop slowly diverges.
    Host.print(F("OK "));
    Host.print((float)pA * DEG_PER_FINE, 5); Host.print(' ');
    Host.println((float)pE * DEG_PER_FINE, 5);
    return;
  }

  if (!strcasecmp(cmd, "VEL")) {
    char *p1 = strtok(NULL, " "), *p2 = strtok(NULL, " ");
    if (!p1 || !p2) { Host.println(F("ERR ARG")); return; }
    noInterrupts();
    enterTrack(az); enterTrack(el);
    az.velCmd = atof(p1) * FINE_PER_DEG;
    el.velCmd = atof(p2) * FINE_PER_DEG;
    interrupts();
    gMode = MODE_TRACK;
    Host.println(F("OK VEL"));
    return;
  }

  if (!strcasecmp(cmd, "STOP")) {
    noInterrupts();
    gVelocityLease.beginStop();
    enterTrack(az); enterTrack(el);
    az.velCmd = 0; el.velCmd = 0;
    interrupts();
    delay(300);                       // let the ramp bring it to rest
    noInterrupts();
    exitTrack(az); exitTrack(el);
    // Keep the expired-session latch if a previously selected low ACC means
    // 300 ms was insufficient. Another explicit STOP is needed after rest.
    if (az.vel == 0.0f && el.vel == 0.0f) gVelocityLease.resetAfterStop();
    interrupts();
    gMode = MODE_HOLD;
    Host.println(F("OK STOP"));
    return;
  }

  if (!strcasecmp(cmd, "EST")) { gEstop = true;  gMode = MODE_FAULT; Host.println(F("OK EST")); return; }
  if (!strcasecmp(cmd, "CLR")) {
    gEstop = false;
    noInterrupts(); exitTrack(az); exitTrack(el); interrupts();
    gMode = MODE_HOLD; Host.println(F("OK CLR")); return;
  }

  if (!strcasecmp(cmd, "ENC")) {
    char *p1 = strtok(NULL, " ");
    if (!p1) { Host.println(F("ERR ARG")); return; }
    gEncEnabled = (atoi(p1) != 0);
    if (!gEncEnabled) { az.encOk = false; el.encOk = false;
                        az.lostSteps = 0; el.lostSteps = 0; }
    Host.print(F("OK ENC ")); Host.println(gEncEnabled ? 1 : 0);
    return;
  }

  if (!strcasecmp(cmd, "DSTAT")) {
    if (strtok(NULL, " ")) { Host.println(F("ERR ARG")); return; }
    // Each accessor such as ot()/s2ga() performs another UART read. Cache
    // DRV_STATUS once per driver so runtime monitoring does not block the
    // velocity command stream on the full diagnostic's repeated reads.
    for (int i = 0; i < 2; i++) {
      TMC2209Stepper &d = (i == 0) ? drvAZ : drvEL;
      const uint32_t status = d.DRV_STATUS();
      Host.print(i == 0 ? F("AZ drvst=0x") : F("EL drvst=0x"));
      Host.print((unsigned long)status, HEX);
      Host.print(F(" en_pin="));
      Host.println(digitalRead(i == 0 ? AZ_EN : EL_EN));
    }
    Host.println(F("OK DSTAT"));
    return;
  }

  if (!strcasecmp(cmd, "DIAG")) {
    // Read the drivers back. UART can be healthy while the motor output
    // stage is dead - the TMC2209's logic runs off 3.3V but its H-bridges
    // need VM (24V). No VM => perfect comms, zero torque.
    for (int i = 0; i < 2; i++) {
      TMC2209Stepper &d = (i == 0) ? drvAZ : drvEL;
      Host.print(i == 0 ? F("AZ ") : F("EL "));
      Host.print(F("conn=")); Host.print(d.test_connection());
      Host.print(F(" toff=")); Host.print(d.toff());
      Host.print(F(" irun=")); Host.print(d.irun());
      Host.print(F(" ihold=")); Host.print(d.ihold());
      Host.print(F(" mres=")); Host.print(d.microsteps());
      Host.print(F(" spread=")); Host.print(d.en_spreadCycle() ? 1 : 0);
      Host.print(F(" drvst=0x")); Host.print((unsigned long)d.DRV_STATUS(), HEX);
      Host.print(F(" ola=")); Host.print(d.ola());
      Host.print(F(" olb=")); Host.print(d.olb());
      Host.print(F(" s2ga=")); Host.print(d.s2ga());
      Host.print(F(" s2gb=")); Host.print(d.s2gb());
      Host.print(F(" ot=")); Host.print(d.ot());
      Host.print(F(" otpw=")); Host.print(d.otpw());
      Host.print(F(" stst=")); Host.print(d.stst());
      Host.print(F(" cs=")); Host.print(d.cs_actual());
      Host.print(F(" en_pin=")); Host.print(digitalRead(i == 0 ? AZ_EN : EL_EN));
      Host.println();
    }
    Host.println(F("OK DIAG"));
    return;
  }

  if (!strcasecmp(cmd, "ACC")) {
    char *p1 = strtok(NULL, " ");
    if (!p1) { Host.println(F("ERR ARG")); return; }
    float a = constrain((float)atof(p1), 1.0f, 500.0f);
    MAX_ACC_DPS2 = a;
    Host.print(F("OK ACC ")); Host.println(a, 2);
    return;
  }

  if (!strcasecmp(cmd, "SPRD")) {
    char *p1 = strtok(NULL, " ");
    if (!p1) { Host.println(F("ERR ARG")); return; }
    bool on = (atoi(p1) != 0);
    drvAZ.en_spreadCycle(on);
    drvEL.en_spreadCycle(on);
    Host.print(F("OK SPRD ")); Host.println(on ? 1 : 0);
    return;
  }

  if (!strcasecmp(cmd, "CUR")) {
    char *p1 = strtok(NULL, " ");
    if (!p1) { Host.println(F("ERR ARG")); return; }
    uint16_t ma = (uint16_t)constrain(atoi(p1), 100, 1200);
    drvAZ.rms_current(ma, (float)HOLD_MULT_PCT / 100.0f);
    drvEL.rms_current(ma, (float)HOLD_MULT_PCT / 100.0f);
    Host.print(F("OK CUR ")); Host.println(ma);
    return;
  }

  if (!strcasecmp(cmd, "LIM")) {
    char *p[4];
    for (int i = 0; i < 4; i++) { p[i] = strtok(NULL, " "); if (!p[i]) { Host.println(F("ERR ARG")); return; } }
    az.softMin = atof(p[0]); az.softMax = atof(p[1]);
    el.softMin = atof(p[2]); el.softMax = atof(p[3]);
    Host.println(F("OK LIM"));
    return;
  }

  Host.println(F("ERR CMD"));
}

// ---------------------------------------------------------------------
//  MAIN LOOP
// ---------------------------------------------------------------------
void loop() {
  // --- serial: USB and UART both accept the same commands ---
  pumpStream(Serial,  lineBuf,  lineLen);
  pumpStream(Serial1, lineBuf1, lineLen1);

  // --- 100 Hz housekeeping: encoders, limits, fault checks ---
  static uint32_t tNext = 0;
  uint32_t now = millis();
  if ((int32_t)(now - tNext) >= 0) {
    tNext = now + 10;

    if (gEncEnabled) {
      axisReadEnc(az);
      axisReadEnc(el);
    }

    // hard limit switches (active low with pullup)
    if (digitalRead(LIM_AZ) == LOW || digitalRead(LIM_EL) == LOW) {
      if (!gEstop) { gEstop = true; gMode = MODE_FAULT; Host.println(F("! LIMIT")); }
    }

    // lost-step / encoder-disagreement watchdog.
    // The encoder cannot see finer than ~250 fine units (its own LSB), so
    // the threshold has to sit well above that or it will false-trip.
    const int32_t LOST_TRIP = 3000;   // ~0.26 deg
    if (gEncEnabled && gMode != MODE_FAULT && az.encOk && el.encOk) {
      if (labs(az.lostSteps) > LOST_TRIP || labs(el.lostSteps) > LOST_TRIP) {
        gEstop = true; gMode = MODE_FAULT;
        Host.println(F("! LOSTSTEP"));
      }
    }

    // encoder dropout
    if (gEncEnabled && (!az.encOk || !el.encOk) && gMode != MODE_FAULT) {
      Host.println(F("! ENCFAIL"));
    }
  }
}
