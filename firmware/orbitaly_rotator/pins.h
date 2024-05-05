// THE per-build file. Everything here is a fact about how *your* board is
// wired, and nothing here is a fact about your rotator.
//
// Gear ratios, travel limits, speeds, backlash, homing — none of that lives on
// the Arduino. The firmware does not know that degrees exist; it emits pulses
// and counts them. If you find yourself wanting to add a constant here that
// describes the *mechanism* rather than the *wiring*, it belongs in the Pi's
// config.yaml instead (PLAN-ARDUINO §3).
//
// These values are reported back in the IDENT handshake, so `orbitaly doctor`
// prints the pinout the board is actually running rather than the one somebody
// wrote down.

#ifndef ORBITALY_PINS_H
#define ORBITALY_PINS_H

#include <stdint.h>

// -- azimuth ---------------------------------------------------------------
#define AZ_STEP_PIN 2
#define AZ_DIR_PIN 3
#define AZ_ENABLE_PIN 4
#define AZ_ENDSTOP_PIN 5   // ORB_PIN_NONE (0xFF) if no switch is fitted

// -- elevation -------------------------------------------------------------
#define EL_STEP_PIN 6
#define EL_DIR_PIN 7
#define EL_ENABLE_PIN 8
#define EL_ENDSTOP_PIN 9

// -- station ---------------------------------------------------------------
#define ESTOP_PIN 10       // ORB_PIN_NONE if none fitted

// Endstops and the E-stop are wired **normally closed** to ground, with the
// input pulled up. Released reads 0; pressed reads 1 — and so does a cut wire,
// a corroded connector, or an unplugged switch. Every wiring failure therefore
// reads as "at the limit" rather than as "clear sky". Do not "fix" this by
// switching to normally-open because the idle logic level looks nicer.
#define ENDSTOP_NORMALLY_CLOSED 1

// A4988 / DRV8825 / TMC2209 all enable on a LOW input.
#define ENABLE_ACTIVE_LOW 1

// Set to 1 for a driver whose DIR sense is backwards from your mechanism. This
// is the *electrical* inversion; the host has its own `invert_dir`, and you
// want exactly one of them set.
#define AZ_INVERT_DIR 0
#define EL_INVERT_DIR 0

// -- driver timing ---------------------------------------------------------

// STEP high time. A4988 and DRV8825 need >= 1 us; TMC2209 is happy with less.
// 5 us is comfortable for all three and costs 0.8% of the CPU at the reference
// azimuth rate of 1600 steps/s.
#define PULSE_WIDTH_US 5

// Settle time after a DIR change before the next STEP edge. Datasheets ask for
// 200 ns; 20 us is free here because a direction reversal is already a
// stopped-motor event.
#define DIR_SETUP_US 20

// Debounce for the *reported* endstop level. The abort reflex is not debounced
// — see the note in orbitaly_rotator.ino.
#define ENDSTOP_DEBOUNCE_MS 5

// -- firmware behaviour ----------------------------------------------------

// No valid frame from the host for this long and the firmware stops trusting
// it. It drains the queue rather than aborting: every plan the host sends
// terminates at rest, so draining stops the motor at a *known* position, which
// an abort would not.
#define FW_WATCHDOG_MS 5000

// After the watchdog has tripped and the queue has drained, drop the enable
// lines. A stepper holding position draws full current and gets hot.
#define FW_IDLE_DISABLE_MS 2000

#define FW_VERSION_MAJOR 1
#define FW_VERSION_MINOR 0
#define FW_VERSION_PATCH 0

#endif  // ORBITALY_PINS_H
