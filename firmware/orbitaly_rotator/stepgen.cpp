#include "stepgen.h"

#include <Arduino.h>

#include "pins.h"
#include "protocol.h"

AzStepGen azGen;
ElStepGen elGen;

// --------------------------------------------------------------------------
// Shared bookkeeping
// --------------------------------------------------------------------------

StepGen::StepGen()
    : inhibitDirection(0),
      stepPin_(ORB_PIN_NONE),
      dirPin_(ORB_PIN_NONE),
      enablePin_(ORB_PIN_NONE),
      invertDir_(false),
      enabled_(false),
      direction_(0),
      remaining_(0),
      requested_(0) {}

void StepGen::begin(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin, bool invertDir) {
  stepPin_ = stepPin;
  dirPin_ = dirPin;
  enablePin_ = enablePin;
  invertDir_ = invertDir;
  if (stepPin_ != ORB_PIN_NONE) {
    pinMode(stepPin_, OUTPUT);
    digitalWrite(stepPin_, LOW);
  }
  if (dirPin_ != ORB_PIN_NONE) {
    pinMode(dirPin_, OUTPUT);
    digitalWrite(dirPin_, LOW);
  }
  if (enablePin_ != ORB_PIN_NONE) {
    pinMode(enablePin_, OUTPUT);
  }
  setEnabled(false);
}

void StepGen::setDirection(int8_t direction) {
  if (dirPin_ == ORB_PIN_NONE || direction == 0) {
    return;
  }
  direction_ = direction;
  bool forward = (direction > 0);
  if (invertDir_) {
    forward = !forward;
  }
  digitalWrite(dirPin_, forward ? HIGH : LOW);
}

void StepGen::setEnabled(bool on) {
  enabled_ = on;
  if (enablePin_ == ORB_PIN_NONE) {
    return;
  }
#if ENABLE_ACTIVE_LOW
  digitalWrite(enablePin_, on ? LOW : HIGH);
#else
  digitalWrite(enablePin_, on ? HIGH : LOW);
#endif
}

void StepGen::start(uint16_t steps, uint32_t periodUs) {
  if (steps == 0 || stepPin_ == ORB_PIN_NONE) {
    return;
  }
  requested_ = steps;
  remaining_ = steps;
  armTimer(periodUs);
}

void StepGen::stop() {
  disarmTimer();
  remaining_ = 0;
}

uint16_t StepGen::stepsDone() const {
  // Read once: `remaining_` is written by the ISR, and on an 8-bit core a
  // 16-bit read can otherwise tear between the two halves.
  uint8_t sreg = SREG;
  cli();
  const uint16_t remaining = remaining_;
  SREG = sreg;
  return (uint16_t)(requested_ - remaining);
}

void StepGen::pulse() {
  // The step is emitted before the count drops, so a stop() racing this ISR can
  // never report a pulse that did not go out.
  digitalWrite(stepPin_, HIGH);
  delayMicroseconds(PULSE_WIDTH_US);
  digitalWrite(stepPin_, LOW);
  remaining_--;
}

void StepGen::onTick() {
  if (remaining_ == 0) {
    disarmTimer();
    return;
  }
  if (inhibitDirection != 0 && direction_ == inhibitDirection) {
    // The switch closed part way through this segment. Stopping here, inside
    // the ISR, is what makes the executed count exact: the host learns the
    // pulse total, not an estimate of it.
    disarmTimer();
    return;
  }
  pulse();
  if (remaining_ == 0) {
    disarmTimer();
  }
}

// --------------------------------------------------------------------------
// Azimuth — Timer1, 16-bit CTC
// --------------------------------------------------------------------------

void AzStepGen::armTimer(uint32_t periodUs) {
  // Pick the smallest prescaler whose 16-bit compare value still spans the
  // period, so the shortest possible tick — and the least quantisation error —
  // is used at every rate.
  static const uint16_t kPrescalers[5] = {1, 8, 64, 256, 1024};
  static const uint8_t kBits[5] = {
      (1 << CS10),
      (1 << CS11),
      (1 << CS11) | (1 << CS10),
      (1 << CS12),
      (1 << CS12) | (1 << CS10),
  };

  uint8_t choice = 4;
  uint32_t compare = 0;
  for (uint8_t i = 0; i < 5; i++) {
    // ticks = periodUs * (F_CPU / 1e6) / prescaler, minus one for CTC.
    const uint32_t ticks = (periodUs * (F_CPU / 1000000UL)) / kPrescalers[i];
    if (ticks <= 65536UL && ticks >= 1UL) {
      choice = i;
      compare = ticks - 1UL;
      break;
    }
  }
  if (compare == 0 && choice == 4) {
    const uint32_t ticks = (periodUs * (F_CPU / 1000000UL)) / 1024UL;
    compare = (ticks > 65536UL) ? 65535UL : (ticks > 0 ? ticks - 1UL : 0UL);
  }

  uint8_t sreg = SREG;
  cli();
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | kBits[choice];  // CTC on OCR1A
  TCNT1 = 0;
  OCR1A = (uint16_t)compare;
  TIFR1 = (1 << OCF1A);   // clear a compare match left over from the last run
  TIMSK1 = (1 << OCIE1A);
  SREG = sreg;
}

void AzStepGen::disarmTimer() {
  TIMSK1 = 0;
  TCCR1B = 0;
}

ISR(TIMER1_COMPA_vect) {
  azGen.onTick();
}

// --------------------------------------------------------------------------
// Elevation — Timer2, 8-bit, with a software postscaler
// --------------------------------------------------------------------------

void ElStepGen::armTimer(uint32_t periodUs) {
  periodUs_ = periodUs;
  accumulatorUs_ = 0;

  uint8_t sreg = SREG;
  cli();
  TCCR2A = (1 << WGM21);              // CTC
  TCCR2B = (1 << CS22) | (1 << CS20); // prescaler 128 -> 8 us per tick at 16 MHz
  TCNT2 = 0;
  OCR2A = (uint8_t)((EL_BASE_TICK_US * (F_CPU / 1000000UL)) / 128UL - 1UL);
  TIFR2 = (1 << OCF2A);
  TIMSK2 = (1 << OCIE2A);
  SREG = sreg;
}

void ElStepGen::disarmTimer() {
  TIMSK2 = 0;
  TCCR2B = 0;
}

void ElStepGen::onBaseTick() {
  if (remaining_ == 0) {
    disarmTimer();
    return;
  }
  accumulatorUs_ += EL_BASE_TICK_US;
  if (accumulatorUs_ < periodUs_) {
    return;
  }
  // Subtract rather than zero: the remainder carries into the next step, so the
  // *average* rate is right even though each individual step is quantised to
  // the base tick. Zeroing here would slow the axis by up to one tick per step.
  accumulatorUs_ -= periodUs_;
  onTick();
}

ISR(TIMER2_COMPA_vect) {
  elGen.onBaseTick();
}
