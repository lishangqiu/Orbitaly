// Step generation. The only file that touches timers, and the only file with
// AVR-specific code in it.
//
// One generator per axis. A generator does exactly one thing: emit N pulses at
// one period and count them as they go out. It has no idea what a trajectory
// is, cannot ramp, and cannot decide to move — all of which live on the Pi.
//
// The count is the product here, not the timing. Jitter costs smoothness and
// acoustic noise; it cannot cost position, because position is derived from
// pulses this class reports as executed and never from elapsed time.

#ifndef ORBITALY_STEPGEN_H
#define ORBITALY_STEPGEN_H

#include <stdint.h>

class StepGen {
 public:
  StepGen();

  void begin(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin, bool invertDir);

  // Emit `steps` pulses at one every `periodUs`. The caller must have set the
  // direction and waited out DIR_SETUP_US already (see setDirection).
  void start(uint16_t steps, uint32_t periodUs);

  // Halt immediately. The executed count is preserved and readable — this is
  // what makes an abort exact, and the whole reason the pulses moved off the
  // Pi's own GPIO. Safe to call from an ISR.
  void stop();

  // Drive the DIR pin. Only legal while stopped; the caller waits DIR_SETUP_US.
  void setDirection(int8_t direction);
  int8_t direction() const { return direction_; }

  void setEnabled(bool on);
  bool enabled() const { return enabled_; }

  bool busy() const { return remaining_ > 0; }
  uint16_t stepsDone() const;
  uint16_t stepsRequested() const { return requested_; }

  // Called from the timer ISR. Public only because the ISR needs it.
  void onTick();

  // Motion in this direction is refused while set (endstop closed). Checked on
  // every step, so a switch that closes mid-segment stops the motor within one
  // step period rather than at the end of the segment.
  volatile int8_t inhibitDirection;

 protected:
  // Timer plumbing differs per axis; the shared bookkeeping does not.
  virtual void armTimer(uint32_t periodUs) = 0;
  virtual void disarmTimer() = 0;

  void pulse();

  uint8_t stepPin_;
  uint8_t dirPin_;
  uint8_t enablePin_;
  bool invertDir_;
  bool enabled_;
  int8_t direction_;
  volatile uint16_t remaining_;
  uint16_t requested_;
};

// Azimuth: Timer1, 16-bit, CTC. A prescaler and OCR1A give an exact period
// anywhere from a few microseconds to seconds, so the only jitter is interrupt
// latency — tens of cycles against a 625 us reference step.
class AzStepGen : public StepGen {
 protected:
  void armTimer(uint32_t periodUs);
  void disarmTimer();
};

// Elevation: Timer2 is only 8-bit, so it runs at a fixed base tick and a
// software accumulator decides which ticks become steps. Jitter is bounded by
// the base tick (~64 us) — about 4% velocity ripple against elevation's 1406 us
// reference period, and zero position error, which is the same trade already
// accepted for lgpio.
class ElStepGen : public StepGen {
 public:
  void onBaseTick();

 protected:
  void armTimer(uint32_t periodUs);
  void disarmTimer();

 private:
  volatile uint32_t accumulatorUs_;
  uint32_t periodUs_;
};

extern AzStepGen azGen;
extern ElStepGen elGen;

// Base tick for the elevation postscaler, in microseconds.
#define EL_BASE_TICK_US 64

#endif  // ORBITALY_STEPGEN_H
