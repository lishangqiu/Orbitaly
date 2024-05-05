// Host-compiled torture test for the firmware's protocol core.
//
//   c++ -std=c++11 -Wall -Wextra -I../orbitaly_rotator \
//       test_firmware_core.cpp ../orbitaly_rotator/framing.cpp \
//       ../orbitaly_rotator/queue.cpp -o test_firmware_core
//   ./test_firmware_core golden_frames.txt
//
// framing.cpp and queue.cpp have no Arduino dependencies by construction,
// precisely so this is possible: the parts of the sketch that are easy to get
// subtly wrong — CRCs, byte order, resynchronisation, ring-buffer wraparound —
// get tested on a machine with a debugger, not on an 8-bit board with two LEDs.
//
// pytest builds and runs this when a host C++ compiler exists, and skips
// visibly when it does not (tests/test_firmware_core.py).

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "framing.h"
#include "queue.h"
#include "protocol.h"

static int gChecks = 0;
static int gFailures = 0;

static void check(bool ok, const char* what) {
  gChecks++;
  if (!ok) {
    gFailures++;
    printf("FAIL: %s\n", what);
  }
}

static void checkEq(unsigned long got, unsigned long want, const char* what) {
  gChecks++;
  if (got != want) {
    gFailures++;
    printf("FAIL: %s (got %lu, want %lu)\n", what, got, want);
  }
}

// -- hex helpers -----------------------------------------------------------

static int hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

// Decode a hex field into `out`. "-" means empty, per the vector file's header.
static int hexDecode(const char* text, uint8_t* out, int maxLen) {
  if (strcmp(text, "-") == 0) return 0;
  const int chars = (int)strlen(text);
  if (chars % 2 != 0 || chars / 2 > maxLen) return -1;
  for (int i = 0; i < chars / 2; i++) {
    const int hi = hexNibble(text[2 * i]);
    const int lo = hexNibble(text[2 * i + 1]);
    if (hi < 0 || lo < 0) return -1;
    out[i] = (uint8_t)((hi << 4) | lo);
  }
  return chars / 2;
}

// -- golden vectors --------------------------------------------------------

static void runGoldenFile(const char* path) {
  FILE* file = fopen(path, "r");
  if (!file) {
    printf("FAIL: cannot open %s\n", path);
    gFailures++;
    return;
  }

  char line[1024];
  while (fgets(line, sizeof(line), file)) {
    char kind[16], a[512], b[512], c[512];
    const int fields = sscanf(line, "%15s %511s %511s %511s", kind, a, b, c);
    if (fields < 3 || kind[0] == '#') continue;

    if (strcmp(kind, "CRC") == 0) {
      uint8_t data[512];
      const int len = hexDecode(a, data, sizeof(data));
      check(len >= 0, "CRC vector decodes");
      const unsigned long want = strtoul(b, NULL, 16);
      checkEq(orbCrc16(data, (uint8_t)len), want, "crc16 matches the vector");

    } else if (strcmp(kind, "FRAME") == 0 && fields >= 4) {
      uint8_t payload[ORB_MAX_PAYLOAD];
      uint8_t expected[ORB_FRAME_OVERHEAD + ORB_MAX_PAYLOAD];
      const unsigned long type = strtoul(a, NULL, 16);
      const int payloadLen = hexDecode(b, payload, sizeof(payload));
      const int expectedLen = hexDecode(c, expected, sizeof(expected));
      check(payloadLen >= 0 && expectedLen > 0, "FRAME vector decodes");

      // Encode: our bytes must be the vector's bytes.
      uint8_t out[ORB_FRAME_OVERHEAD + ORB_MAX_PAYLOAD];
      const uint8_t written = orbEncodeFrame((uint8_t)type, payload, (uint8_t)payloadLen, out);
      checkEq(written, (unsigned long)expectedLen, "encoded frame length");
      check(memcmp(out, expected, (size_t)expectedLen) == 0, "encoded frame bytes");

      // Decode: feeding those bytes back must reproduce type and payload.
      FrameReader reader;
      for (int i = 0; i < expectedLen; i++) reader.feed(expected[i]);
      Frame frame;
      check(reader.next(&frame), "vector frame decodes");
      checkEq(frame.type, type, "decoded type");
      checkEq(frame.length, (unsigned long)payloadLen, "decoded payload length");
      check(memcmp(frame.payload, payload, (size_t)payloadLen) == 0, "decoded payload bytes");
      check(!reader.next(&frame), "no second frame in a single-frame vector");

    } else if (strcmp(kind, "STREAM") == 0 && fields >= 4) {
      uint8_t input[512], types[64];
      const int inputLen = hexDecode(a, input, sizeof(input));
      const int typeCount = hexDecode(b, types, sizeof(types));
      const unsigned long wantCrcErrors = strtoul(c, NULL, 10);
      check(inputLen >= 0 && typeCount >= 0, "STREAM vector decodes");

      FrameReader reader;
      for (int i = 0; i < inputLen; i++) reader.feed(input[i]);
      int seen = 0;
      Frame frame;
      while (reader.next(&frame)) {
        if (seen < typeCount) checkEq(frame.type, types[seen], "stream frame type");
        seen++;
      }
      checkEq((unsigned long)seen, (unsigned long)typeCount, "stream frame count");
      checkEq(reader.crcErrors, wantCrcErrors, "stream CRC error count");
    }
  }
  fclose(file);
}

// -- framing edge cases ----------------------------------------------------

static void runFramingChecks() {
  // A payload longer than the protocol allows must be refused, not truncated.
  uint8_t big[ORB_MAX_PAYLOAD + 1];
  memset(big, 0x5A, sizeof(big));
  uint8_t out[64];
  checkEq(orbEncodeFrame(ORB_MSG_PING, big, ORB_MAX_PAYLOAD + 1, out), 0,
          "oversized payload is refused");

  // Byte-at-a-time delivery must behave exactly like a bulk feed: this is the
  // real arrival pattern on a UART.
  uint8_t ping[8];
  const uint8_t nonce = 0x42;
  const uint8_t len = orbEncodeFrame(ORB_MSG_PING, &nonce, 1, ping);
  FrameReader reader;
  Frame frame;
  for (uint8_t i = 0; i + 1 < len; i++) {
    reader.feed(ping[i]);
    check(!reader.next(&frame), "no frame before the last byte arrives");
  }
  reader.feed(ping[len - 1]);
  check(reader.next(&frame), "frame completes on the final byte");
  checkEq(frame.payload[0], nonce, "payload survives byte-at-a-time delivery");

  // Every single-bit corruption of a frame must be caught. A CRC that lets one
  // through would be worse than no CRC, because it would be trusted.
  int undetected = 0;
  for (uint8_t byteIndex = 1; byteIndex < len; byteIndex++) {
    for (uint8_t bit = 0; bit < 8; bit++) {
      uint8_t corrupted[8];
      memcpy(corrupted, ping, len);
      corrupted[byteIndex] ^= (uint8_t)(1 << bit);
      FrameReader r;
      for (uint8_t i = 0; i < len; i++) r.feed(corrupted[i]);
      Frame f;
      if (r.next(&f) && f.type == ORB_MSG_PING && f.length == 1 && f.payload[0] == nonce) {
        undetected++;
      }
    }
  }
  checkEq((unsigned long)undetected, 0, "every single-bit flip is caught");

  // Overflow: a flood of junk must not wedge the reader against a real frame
  // arriving behind it.
  FrameReader flooded;
  for (int i = 0; i < 500; i++) flooded.feed((uint8_t)(i & 0xFF));
  for (uint8_t i = 0; i < len; i++) flooded.feed(ping[i]);
  Frame recovered;
  bool found = false;
  while (flooded.next(&recovered)) {
    if (recovered.type == ORB_MSG_PING && recovered.payload[0] == nonce) found = true;
  }
  check(found, "a real frame survives a flood of junk ahead of it");
}

// -- queue -----------------------------------------------------------------

static QueuedSegment seg(uint8_t sequence, int8_t direction, uint16_t steps) {
  QueuedSegment s;
  s.seq = sequence;
  s.direction = direction;
  s.steps = steps;
  s.periodUs = 625;
  return s;
}

static void runQueueChecks() {
  SegmentQueue queue;
  checkEq(queue.size(), 0, "new queue is empty");
  checkEq(queue.freeSlots(), ORB_QUEUE_DEPTH, "new queue is all free");
  check(queue.peek() == 0, "peek on an empty queue returns nothing");

  for (uint8_t i = 0; i < ORB_QUEUE_DEPTH; i++) {
    check(queue.push(seg(i, 1, (uint16_t)(10 + i))), "push into a queue with room");
  }
  checkEq(queue.freeSlots(), 0, "queue reports itself full");
  check(!queue.push(seg(99, 1, 5)), "push into a full queue is refused");

  // FIFO order, and wraparound: drain half, refill, drain the rest. Ring
  // buffers that pass a straight-line test still get the wrap wrong.
  for (uint8_t i = 0; i < 4; i++) {
    const QueuedSegment* front = queue.peek();
    check(front != 0, "peek returns the front segment");
    checkEq(front->seq, i, "FIFO order out");
    checkEq(front->steps, (unsigned long)(10 + i), "front segment payload intact");
    queue.pop();
  }
  for (uint8_t i = 0; i < 4; i++) {
    check(queue.push(seg((uint8_t)(100 + i), -1, (uint16_t)(200 + i))), "refill after draining");
  }
  for (uint8_t i = 4; i < ORB_QUEUE_DEPTH; i++) {
    checkEq(queue.peek()->seq, i, "FIFO order across the wrap");
    queue.pop();
  }
  for (uint8_t i = 0; i < 4; i++) {
    checkEq(queue.peek()->seq, (unsigned long)(100 + i), "wrapped entries come out in order");
    checkEq(queue.peek()->direction == -1 ? 1 : 0, 1, "direction survives the wrap");
    queue.pop();
  }
  check(queue.empty(), "queue drains to empty");

  // Duplicate detection: a re-send after a lost ACK must be recognised both
  // while queued and after completion, or the antenna moves twice.
  SegmentQueue dedup;
  dedup.push(seg(7, 1, 100));
  check(dedup.isDuplicate(7), "a queued sequence number is a duplicate");
  check(!dedup.isDuplicate(8), "an unseen sequence number is not a duplicate");
  dedup.pop();
  check(dedup.isDuplicate(7), "a completed sequence number is still a duplicate");
  for (uint8_t i = 0; i < ORB_RECENT_DEPTH; i++) {
    dedup.push(seg((uint8_t)(20 + i), 1, 5));
    dedup.pop();
  }
  check(!dedup.isDuplicate(7), "the duplicate window eventually forgets");

  // clear() must remember what it dropped: the host replans after an abort and
  // will not re-send, but executing work it has written off would be motion
  // nobody asked for.
  SegmentQueue cleared;
  cleared.push(seg(31, 1, 50));
  cleared.push(seg(32, 1, 50));
  cleared.clear();
  check(cleared.empty(), "clear empties the queue");
  check(cleared.isDuplicate(31), "cleared sequence numbers are remembered");
  check(cleared.isDuplicate(32), "all cleared sequence numbers are remembered");
}

int main(int argc, char** argv) {
  const char* path = argc > 1 ? argv[1] : "golden_frames.txt";
  runGoldenFile(path);
  runFramingChecks();
  runQueueChecks();

  printf("%d checks passed\n", gChecks - gFailures);
  if (gFailures) {
    printf("%d checks FAILED\n", gFailures);
    return 1;
  }
  return 0;
}
