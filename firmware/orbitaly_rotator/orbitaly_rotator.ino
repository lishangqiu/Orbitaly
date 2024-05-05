// Orbitaly rotator firmware — an exact pulse executor, and nothing more.
//
// This is deliberately the opposite of K3NG-style rotator firmware. Those
// accept position targets ("go to az 137") and do their own ramping and
// pursuit, which is why such setups lurch: the controller re-decides the
// trajectory every time a new bearing lands, has no committed queue to replan
// from, and reports position by its own reckoning that the host cannot audit.
//
// Here, all trajectory intelligence lives on the Raspberry Pi, tested by a suite
// this board never runs. The Arduino executes segments — "N pulses in this
// direction at this period" — and reports exactly how many went out. It does
// not know degrees exist.
//
// WHAT THIS FIRMWARE DOES:
//   execute segments in order with exact counts; serialise direction changes;
//   watch endstops and the E-stop at interrupt latency, abort locally, and
//   report the exact pulse count at the cut; manage enable lines; answer
//   status queries; enforce a bounded queue with explicit flow control.
//
// WHAT IT MUST NEVER DO:
//   generate trajectories, ramp, interpolate or smooth; accept position
//   targets; guess a position; drop or reorder segments silently; move on its
//   own initiative — including "helpfully" backing off an endstop; hold
//   configuration the host reasons about (gear ratios, travel limits and speeds
//   live on the Pi).
//
// Flashing:
//   arduino-cli compile --fqbn arduino:avr:uno firmware/orbitaly_rotator
//   arduino-cli upload  --fqbn arduino:avr:uno -p /dev/ttyACM0 firmware/orbitaly_rotator
//
// See firmware/README.md for wiring and commissioning.

#include "framing.h"
#include "pins.h"
#include "protocol.h"
#include "queue.h"
#include "stepgen.h"

// --------------------------------------------------------------------------
// State
// --------------------------------------------------------------------------

struct Axis {
  SegmentQueue queue;
  StepGen* gen;
  uint8_t endstopPin;

  bool running;
  uint8_t currentSeq;
  int8_t currentDir;

  // Endstop debouncing. The *reported* level is debounced; the abort reflex is
  // not — see onEndstopEdge.
  uint8_t rawLevel;
  uint8_t reportedLevel;
  unsigned long lastEdgeMs;
};

static Axis axes[ORB_AXIS_COUNT];
static FrameReader reader;

static bool gReady = false;             // set by a version-matched HELLO
static unsigned long gLastHostFrameMs = 0;
static unsigned long gLastIdentMs = 0;
static unsigned long gDrainedAtMs = 0;
static bool gWatchdogTripped = false;
static uint8_t gEstopLevel = 0;
static uint8_t gEstopReported = 0;

// --------------------------------------------------------------------------
// Sending
// --------------------------------------------------------------------------

static void sendFrame(uint8_t type, const uint8_t* payload, uint8_t length) {
  uint8_t out[ORB_FRAME_OVERHEAD + ORB_MAX_PAYLOAD];
  const uint8_t n = orbEncodeFrame(type, payload, length, out);
  if (n) {
    Serial.write(out, n);
  }
}

static void put16(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)(v >> 8);
}

static uint32_t get32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

static void sendIdent() {
  uint8_t payload[ORB_LEN_IDENT];
  payload[0] = ORB_PROTOCOL_VERSION;
  payload[1] = FW_VERSION_MAJOR;
  payload[2] = FW_VERSION_MINOR;
  payload[3] = FW_VERSION_PATCH;
  payload[4] = ORB_QUEUE_DEPTH;
  payload[5] = ORB_AXIS_COUNT;

  uint16_t caps = ORB_CAP_EXACT_ABORT;
#if ENDSTOP_NORMALLY_CLOSED
  caps |= ORB_CAP_ENDSTOP_NC;
#endif
#if ENABLE_ACTIVE_LOW
  caps |= ORB_CAP_ENABLE_ACTIVE_LOW;
#endif
  if (ESTOP_PIN != ORB_PIN_NONE) caps |= ORB_CAP_ESTOP_FITTED;
  if (AZ_ENDSTOP_PIN != ORB_PIN_NONE) caps |= ORB_CAP_AZ_ENDSTOP;
  if (EL_ENDSTOP_PIN != ORB_PIN_NONE) caps |= ORB_CAP_EL_ENDSTOP;
  put16(&payload[6], caps);
  put16(&payload[8], DIR_SETUP_US);
  payload[10] = PULSE_WIDTH_US;
  payload[11] = ENDSTOP_DEBOUNCE_MS;
  payload[12] = AZ_STEP_PIN;
  payload[13] = AZ_DIR_PIN;
  payload[14] = AZ_ENABLE_PIN;
  payload[15] = AZ_ENDSTOP_PIN;
  payload[16] = EL_STEP_PIN;
  payload[17] = EL_DIR_PIN;
  payload[18] = EL_ENABLE_PIN;
  payload[19] = EL_ENDSTOP_PIN;
  sendFrame(ORB_MSG_IDENT, payload, ORB_LEN_IDENT);
  gLastIdentMs = millis();
}

static void sendAck(uint8_t axis, uint8_t seq, uint8_t slotsFree) {
  const uint8_t payload[ORB_LEN_ACK] = {axis, seq, slotsFree};
  sendFrame(ORB_MSG_ACK, payload, ORB_LEN_ACK);
}

static void sendNack(uint8_t axis, uint8_t seq, uint8_t reason) {
  const uint8_t payload[ORB_LEN_NACK] = {axis, seq, reason};
  sendFrame(ORB_MSG_NACK, payload, ORB_LEN_NACK);
}

static void sendDone(uint8_t axis, uint8_t seq, uint16_t steps) {
  uint8_t payload[ORB_LEN_DONE];
  payload[0] = axis;
  payload[1] = seq;
  put16(&payload[2], steps);
  sendFrame(ORB_MSG_DONE, payload, ORB_LEN_DONE);
}

static void sendAborted(uint8_t axis, uint8_t seq, uint16_t stepsDone, uint8_t cause) {
  uint8_t payload[ORB_LEN_ABORTED];
  payload[0] = axis;
  payload[1] = seq;
  put16(&payload[2], stepsDone);
  payload[4] = cause;
  sendFrame(ORB_MSG_ABORTED, payload, ORB_LEN_ABORTED);
}

static void sendEvent(uint8_t kind, uint8_t state) {
  const uint8_t payload[ORB_LEN_EVENT] = {kind, state};
  sendFrame(ORB_MSG_EVENT, payload, ORB_LEN_EVENT);
}

static void sendState() {
  uint8_t payload[1 + 3 * ORB_AXIS_COUNT + 1];
  payload[0] = ORB_AXIS_COUNT;
  for (uint8_t i = 0; i < ORB_AXIS_COUNT; i++) {
    Axis& axis = axes[i];
    uint8_t flags = 0;
    if (axis.gen->inhibitDirection < 0) flags |= ORB_FLAG_INHIBIT_NEG;
    if (axis.gen->inhibitDirection > 0) flags |= ORB_FLAG_INHIBIT_POS;
    if (axis.gen->enabled()) flags |= ORB_FLAG_ENABLED;
    payload[1 + 3 * i] = (uint8_t)(axis.queue.size() + (axis.running ? 1 : 0));
    payload[2 + 3 * i] = flags;
    payload[3 + 3 * i] = axis.reportedLevel;
  }
  payload[1 + 3 * ORB_AXIS_COUNT] = gEstopReported;
  sendFrame(ORB_MSG_STATE, payload, (uint8_t)sizeof(payload));
}

// --------------------------------------------------------------------------
// Endstops and E-stop
// --------------------------------------------------------------------------

static uint8_t readSwitch(uint8_t pin) {
  if (pin == ORB_PIN_NONE) {
    return 0;
  }
  const int level = digitalRead(pin);
#if ENDSTOP_NORMALLY_CLOSED
  // NC to ground with a pull-up: released reads LOW, pressed reads HIGH — and so
  // does a cut wire or an unplugged connector.
  return (level == HIGH) ? 1 : 0;
#else
  return (level == LOW) ? 1 : 0;
#endif
}

// Pin-change interrupts cover the whole switch group; the handler works out
// which pins moved.
//
// The abort reflex here is deliberately NOT debounced. A spurious edge that
// stops the motor costs one stopped segment and a replan; a debounce delay that
// lets the carriage keep driving into a closed switch costs a gearbox. The
// debounce applies only to the level *reported* to the host, where flapping
// would be noise rather than danger.
static void serviceSwitchEdges() {
  const unsigned long now = millis();
  for (uint8_t i = 0; i < ORB_AXIS_COUNT; i++) {
    Axis& axis = axes[i];
    if (axis.endstopPin == ORB_PIN_NONE) {
      continue;
    }
    const uint8_t level = readSwitch(axis.endstopPin);
    if (level != axis.rawLevel) {
      axis.rawLevel = level;
      axis.lastEdgeMs = now;
    }
    // React to a closure at once, whatever the debounce says.
    axis.gen->inhibitDirection = level ? -1 : 0;
    if (level && axis.running && axis.gen->direction() < 0) {
      axis.gen->stop();
    }
    if (level != axis.reportedLevel && (now - axis.lastEdgeMs) >= ENDSTOP_DEBOUNCE_MS) {
      axis.reportedLevel = level;
      sendEvent(i == ORB_AXIS_AZ ? ORB_EVENT_ENDSTOP_AZ : ORB_EVENT_ENDSTOP_EL, level);
    }
  }
}

static void dropQueue(Axis& axis) {
  axis.queue.clear();
}

static void abortAxis(uint8_t index, uint8_t cause) {
  Axis& axis = axes[index];
  if (axis.running) {
    axis.gen->stop();
    const uint16_t done = axis.gen->stepsDone();
    axis.running = false;
    sendAborted(index, axis.currentSeq, done, cause);
  } else {
    sendAborted(index, ORB_SEQ_NONE, 0, cause);
  }
  dropQueue(axis);
}

static void serviceEstop() {
  if (ESTOP_PIN == ORB_PIN_NONE) {
    return;
  }
  const uint8_t level = readSwitch(ESTOP_PIN);
  if (level == gEstopLevel) {
    return;
  }
  gEstopLevel = level;
  gEstopReported = level;
  sendEvent(ORB_EVENT_ESTOP, level);
  if (!level) {
    return;
  }
  for (uint8_t i = 0; i < ORB_AXIS_COUNT; i++) {
    abortAxis(i, ORB_ABORT_ESTOP);
    // Enables drop too. This is why an E-stop still costs the position
    // reference where an endstop hit does not: the count is exact either way,
    // but an unpowered stepper under wind load holds nothing.
    axes[i].gen->setEnabled(false);
  }
}

// --------------------------------------------------------------------------
// Segment execution
// --------------------------------------------------------------------------

static void serviceAxis(uint8_t index) {
  Axis& axis = axes[index];

  if (axis.running) {
    if (axis.gen->busy()) {
      return;
    }
    const uint16_t done = axis.gen->stepsDone();
    axis.running = false;
    if (done < axis.gen->stepsRequested()) {
      // Cut short: the only thing that stops a generator early is the endstop
      // reflex, and it knows exactly how far it got.
      sendAborted(index, axis.currentSeq, done, ORB_ABORT_ENDSTOP);
      dropQueue(axis);
      return;
    }
    sendDone(index, axis.currentSeq, done);
  }

  const QueuedSegment* next = axis.queue.peek();
  if (next == 0) {
    return;
  }
  if (gEstopLevel) {
    return;  // nothing starts while the mushroom is down
  }
  if (axis.gen->inhibitDirection != 0 && next->direction == axis.gen->inhibitDirection) {
    return;  // into a closed switch; the host will replan
  }

  if (next->direction != axis.gen->direction()) {
    // Drain, flip DIR, wait out the driver's setup time. Doing this here is why
    // the host does not have to time a 20 us window across a 1-2 ms USB link.
    axis.gen->setDirection(next->direction);
    delayMicroseconds(DIR_SETUP_US);
  }
  axis.currentSeq = next->seq;
  axis.currentDir = next->direction;
  axis.running = true;
  axis.gen->start(next->steps, next->periodUs);
  axis.queue.pop();
}

// --------------------------------------------------------------------------
// Frame dispatch
// --------------------------------------------------------------------------

static void onSeg(const Frame& frame) {
  const uint8_t index = frame.payload[0];
  const uint8_t seq = frame.payload[1];
  if (index >= ORB_AXIS_COUNT) {
    sendNack(index, seq, ORB_NACK_BAD_AXIS);
    return;
  }
  Axis& axis = axes[index];

  if (axis.queue.isDuplicate(seq) || (axis.running && axis.currentSeq == seq)) {
    // A re-send after a lost ACK. Acknowledge it again; executing it twice
    // would move the antenna by a segment nobody asked for.
    sendAck(index, seq, axis.queue.freeSlots());
    return;
  }

  QueuedSegment segment;
  segment.seq = seq;
  segment.direction = (int8_t)frame.payload[2];
  segment.steps = (uint16_t)frame.payload[3] | ((uint16_t)frame.payload[4] << 8);
  segment.periodUs = get32(&frame.payload[5]);

  if (segment.direction != 1 && segment.direction != -1) {
    sendNack(index, seq, ORB_NACK_BAD_LENGTH);
    return;
  }
  if (axis.gen->inhibitDirection != 0 && segment.direction == axis.gen->inhibitDirection) {
    sendNack(index, seq, ORB_NACK_INHIBITED);
    return;
  }
  if (!axis.queue.push(segment)) {
    // Refused, never silently dropped: a dropped segment is a position error
    // the host would have no way of learning about.
    sendNack(index, seq, ORB_NACK_QUEUE_FULL);
    return;
  }
  sendAck(index, seq, axis.queue.freeSlots());
}

static void dispatch(const Frame& frame) {
  gLastHostFrameMs = millis();
  gWatchdogTripped = false;

  if (frame.type == ORB_MSG_HELLO) {
    if (frame.length != ORB_LEN_HELLO || frame.payload[0] != ORB_PROTOCOL_VERSION) {
      sendNack(ORB_AXIS_ALL, ORB_SEQ_NONE, ORB_NACK_BAD_VERSION);
      return;
    }
    gReady = true;
    sendIdent();
    return;
  }

  if (!gReady) {
    // Anything before a version-matched HELLO is refused. A host that skipped
    // the handshake has not proved it speaks this protocol, and guessing is how
    // a stale flash moves an antenna.
    sendNack(ORB_AXIS_ALL, ORB_SEQ_NONE, ORB_NACK_NOT_READY);
    return;
  }

  switch (frame.type) {
    case ORB_MSG_SEG:
      if (frame.length != ORB_LEN_SEG) {
        sendNack(ORB_AXIS_ALL, ORB_SEQ_NONE, ORB_NACK_BAD_LENGTH);
        return;
      }
      onSeg(frame);
      return;

    case ORB_MSG_ABORT: {
      const uint8_t target = frame.payload[0];
      for (uint8_t i = 0; i < ORB_AXIS_COUNT; i++) {
        if (target == ORB_AXIS_ALL || target == i) {
          abortAxis(i, ORB_ABORT_HOST);
        }
      }
      return;
    }

    case ORB_MSG_ENABLE: {
      const uint8_t target = frame.payload[0];
      const bool on = frame.payload[1] != 0;
      for (uint8_t i = 0; i < ORB_AXIS_COUNT; i++) {
        if (target == ORB_AXIS_ALL || target == i) {
          axes[i].gen->setEnabled(on);
        }
      }
      return;
    }

    case ORB_MSG_CLEAR_FAULT: {
      // Mirrors the host's HTTP 409: refused while the condition is still
      // present. There is no latched state to clear here — the inhibit tracks
      // the live switch — so this is purely an answer to "may I?".
      const uint8_t target = frame.payload[0];
      for (uint8_t i = 0; i < ORB_AXIS_COUNT; i++) {
        if (target != ORB_AXIS_ALL && target != i) {
          continue;
        }
        if (axes[i].reportedLevel || gEstopLevel) {
          sendNack(i, ORB_SEQ_NONE, ORB_NACK_INHIBITED);
        }
      }
      return;
    }

    case ORB_MSG_PING: {
      const uint8_t payload[ORB_LEN_PONG] = {frame.payload[0]};
      sendFrame(ORB_MSG_PONG, payload, ORB_LEN_PONG);
      return;
    }

    case ORB_MSG_STATUS:
      sendState();
      return;

    default:
      sendNack(ORB_AXIS_ALL, ORB_SEQ_NONE, ORB_NACK_BAD_TYPE);
      return;
  }
}

// --------------------------------------------------------------------------
// Watchdog
// --------------------------------------------------------------------------

static void serviceWatchdog() {
  if (!gReady) {
    return;
  }
  const unsigned long now = millis();
  if ((now - gLastHostFrameMs) <= FW_WATCHDOG_MS) {
    return;
  }
  gWatchdogTripped = true;

  // Deliberately not an abort. Every plan the host sends terminates at rest, so
  // draining what is queued stops the motor at a position the host can still
  // account for. Aborting would lose that and gain nothing.
  bool idle = true;
  for (uint8_t i = 0; i < ORB_AXIS_COUNT; i++) {
    if (axes[i].running || !axes[i].queue.empty()) {
      idle = false;
    }
  }
  if (!idle) {
    gDrainedAtMs = now;
    return;
  }
  if ((now - gDrainedAtMs) >= FW_IDLE_DISABLE_MS) {
    for (uint8_t i = 0; i < ORB_AXIS_COUNT; i++) {
      if (axes[i].gen->enabled()) {
        axes[i].gen->setEnabled(false);
      }
    }
  }
}

// --------------------------------------------------------------------------
// Arduino entry points
// --------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);

  axes[ORB_AXIS_AZ].gen = &azGen;
  axes[ORB_AXIS_AZ].endstopPin = AZ_ENDSTOP_PIN;
  axes[ORB_AXIS_EL].gen = &elGen;
  axes[ORB_AXIS_EL].endstopPin = EL_ENDSTOP_PIN;
  for (uint8_t i = 0; i < ORB_AXIS_COUNT; i++) {
    axes[i].running = false;
    axes[i].currentSeq = ORB_SEQ_NONE;
    axes[i].currentDir = 0;
    axes[i].rawLevel = 0;
    axes[i].reportedLevel = 0;
    axes[i].lastEdgeMs = 0;
    if (axes[i].endstopPin != ORB_PIN_NONE) {
      pinMode(axes[i].endstopPin, INPUT_PULLUP);
    }
  }
  if (ESTOP_PIN != ORB_PIN_NONE) {
    pinMode(ESTOP_PIN, INPUT_PULLUP);
  }

  azGen.begin(AZ_STEP_PIN, AZ_DIR_PIN, AZ_ENABLE_PIN, AZ_INVERT_DIR);
  elGen.begin(EL_STEP_PIN, EL_DIR_PIN, EL_ENABLE_PIN, EL_INVERT_DIR);

  // Read the switches before announcing, so the first STATE the host sees is
  // the truth rather than an optimistic default.
  for (uint8_t i = 0; i < ORB_AXIS_COUNT; i++) {
    axes[i].rawLevel = axes[i].reportedLevel = readSwitch(axes[i].endstopPin);
    axes[i].gen->inhibitDirection = axes[i].reportedLevel ? -1 : 0;
  }
  gEstopLevel = gEstopReported = readSwitch(ESTOP_PIN);

  gLastHostFrameMs = millis();
  gDrainedAtMs = gLastHostFrameMs;
  sendIdent();
}

void loop() {
  while (Serial.available() > 0) {
    reader.feed((uint8_t)Serial.read());
  }
  Frame frame;
  while (reader.next(&frame)) {
    dispatch(frame);
  }

  serviceEstop();
  serviceSwitchEdges();
  for (uint8_t i = 0; i < ORB_AXIS_COUNT; i++) {
    serviceAxis(i);
  }
  serviceWatchdog();

  // Repeat the boot announcement until the host says something. Opening the
  // port resets this board, so the host is often still settling its own end
  // when the first IDENT goes out; announcing once would lose that race and the
  // host would sit through its whole connect timeout for no reason.
  if (!gReady && (millis() - gLastIdentMs) >= 1000UL) {
    sendIdent();
  }
}
