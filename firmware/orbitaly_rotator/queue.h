// Per-axis segment ring. No Arduino dependencies — compiled and tortured by
// the host test suite alongside framing.cpp.
//
// The queue holds work the host has handed over and the step generator has not
// finished. It is deliberately shallow (ORB_QUEUE_DEPTH): deep buffering would
// mean a retarget waits behind stale motion, and the whole point of planning on
// the Pi is that the Pi can change its mind.

#ifndef ORBITALY_QUEUE_H
#define ORBITALY_QUEUE_H

#include <stdint.h>

#include "protocol.h"

// Eight segments is ~0.16 s of motion at the worst-case segment rate and much
// more in practice, since ramp segments merge. Sized against the host's ~100 ms
// commitment window, not against available SRAM.
#define ORB_QUEUE_DEPTH 8

// How many finished sequence numbers to remember for duplicate detection. A
// re-send after a lost ACK is the realistic case, and the host never has more
// than ORB_QUEUE_DEPTH outstanding, so a window of that size cannot miss one.
#define ORB_RECENT_DEPTH 8

struct QueuedSegment {
  uint8_t seq;
  int8_t direction;
  uint16_t steps;
  uint32_t periodUs;
};

class SegmentQueue {
 public:
  SegmentQueue();

  // Accept a segment. Returns false when full — the caller NACKs with
  // QUEUE_FULL rather than dropping it silently, because a silently dropped
  // segment is a position error the host would never learn about.
  bool push(const QueuedSegment& segment);

  // Oldest queued segment, or 0 when empty. Valid until the next pop/clear.
  const QueuedSegment* peek() const;

  // Retire the oldest segment and remember its sequence number as completed.
  void pop();

  // Drop everything queued. Used by abort: the cut segment is reported
  // separately, with its exact executed count.
  void clear();

  uint8_t size() const { return count_; }
  bool empty() const { return count_ == 0; }
  uint8_t freeSlots() const { return (uint8_t)(ORB_QUEUE_DEPTH - count_); }

  // True when this sequence number is already queued or recently completed —
  // i.e. this SEG is a re-send after a lost ACK and must be re-ACKed, not
  // executed twice. Idempotent re-sends are what make the CRC/NACK recovery
  // path safe for position.
  bool isDuplicate(uint8_t seq) const;

 private:
  void remember(uint8_t seq);

  QueuedSegment slots_[ORB_QUEUE_DEPTH];
  uint8_t head_;
  uint8_t count_;

  uint8_t recent_[ORB_RECENT_DEPTH];
  uint8_t recentCount_;
  uint8_t recentNext_;
};

#endif  // ORBITALY_QUEUE_H
