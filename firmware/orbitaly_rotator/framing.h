// Frame encode/decode. No Arduino dependencies — the host test suite compiles
// this file directly and feeds it the same golden vectors the Python tests use.

#ifndef ORBITALY_FRAMING_H
#define ORBITALY_FRAMING_H

#include <stdint.h>

#include "protocol.h"

struct Frame {
  uint8_t type;
  uint8_t length;
  uint8_t payload[ORB_MAX_PAYLOAD];
};

// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final xor.
uint16_t orbCrc16(const uint8_t* data, uint8_t length);

// Write a complete frame into `out`, which must hold ORB_FRAME_OVERHEAD +
// length bytes. Returns the number written, or 0 if the payload is too long.
uint8_t orbEncodeFrame(uint8_t type, const uint8_t* payload, uint8_t length, uint8_t* out);

// Streaming reassembler. Mirrors the Python FrameDecoder byte for byte,
// including its resynchronisation rule: on a bad CRC or an impossible length,
// drop one byte and rescan for the next start byte, so a corrupted length
// field costs one frame rather than the link.
class FrameReader {
 public:
  FrameReader();

  // Append one received byte. Never blocks, never allocates.
  void feed(uint8_t byte);

  // Extract the next complete frame, if the buffer holds one. Call in a loop
  // until it returns false — one fed byte can reveal more than one frame when
  // a false start byte was hiding a real one behind it.
  bool next(Frame* out);

  void reset();

  uint8_t pending() const { return len_; }

  // Kept because a link that works but resyncs constantly looks exactly like a
  // healthy one unless somebody counts. `selftest --serial` reports these.
  uint16_t crcErrors;
  uint16_t resyncs;
  uint16_t overflows;

 private:
  // Two maximum frames: enough that a resync never discards a frame that had
  // already arrived intact behind a false start byte.
  static const uint8_t CAPACITY = 2 * (ORB_FRAME_OVERHEAD + ORB_MAX_PAYLOAD);

  enum Parse { PARSE_OK, PARSE_INCOMPLETE, PARSE_BAD };

  // Try to read a frame at `offset` without mutating the buffer. `out` and
  // `total` may be null when only the verdict is wanted.
  Parse parseAt(uint8_t offset, Frame* out, uint8_t* total) const;

  // Offset of a later start byte that parses cleanly, or -1 if there is none
  // (or not enough bytes yet to tell).
  int16_t recoverFromLaterSof() const;

  void dropFront(uint8_t count);

  uint8_t buf_[CAPACITY];
  uint8_t len_;
};

#endif  // ORBITALY_FRAMING_H
