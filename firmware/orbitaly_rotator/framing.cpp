#include "framing.h"

#include <string.h>

uint16_t orbCrc16(const uint8_t* data, uint8_t length) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x8000) {
        crc = (uint16_t)((crc << 1) ^ 0x1021);
      } else {
        crc = (uint16_t)(crc << 1);
      }
    }
  }
  return crc;
}

uint8_t orbEncodeFrame(uint8_t type, const uint8_t* payload, uint8_t length, uint8_t* out) {
  if (length > ORB_MAX_PAYLOAD) {
    return 0;
  }
  out[0] = ORB_SOF;
  out[1] = length;
  out[2] = type;
  for (uint8_t i = 0; i < length; i++) {
    out[3 + i] = payload[i];
  }
  // The CRC covers everything but the start byte, which carries no information.
  const uint16_t crc = orbCrc16(&out[1], (uint8_t)(length + 2));
  out[3 + length] = (uint8_t)(crc & 0xFF);
  out[4 + length] = (uint8_t)(crc >> 8);
  return (uint8_t)(ORB_FRAME_OVERHEAD + length);
}

FrameReader::FrameReader() : crcErrors(0), resyncs(0), overflows(0), len_(0) {}

void FrameReader::reset() {
  len_ = 0;
}

void FrameReader::dropFront(uint8_t count) {
  if (count >= len_) {
    len_ = 0;
    return;
  }
  memmove(buf_, buf_ + count, (size_t)(len_ - count));
  len_ = (uint8_t)(len_ - count);
}

void FrameReader::feed(uint8_t byte) {
  if (len_ >= CAPACITY) {
    // Full and still nothing parseable: the oldest byte cannot be the start of
    // a real frame, so it goes. Losing the oldest keeps the newest, which is
    // the half more likely to resynchronise.
    overflows++;
    dropFront(1);
  }
  buf_[len_++] = byte;
}

FrameReader::Parse FrameReader::parseAt(uint8_t offset, Frame* out, uint8_t* total) const {
  const uint8_t available = (uint8_t)(len_ - offset);
  if (available < 3) {
    return PARSE_INCOMPLETE;  // need SOF, len, type before the length is known
  }
  const uint8_t length = buf_[offset + 1];
  if (length > ORB_MAX_PAYLOAD) {
    return PARSE_BAD;
  }
  const uint8_t want = (uint8_t)(ORB_FRAME_OVERHEAD + length);
  if (available < want) {
    return PARSE_INCOMPLETE;
  }
  const uint16_t crcRx =
      (uint16_t)buf_[offset + 3 + length] | ((uint16_t)buf_[offset + 4 + length] << 8);
  if (orbCrc16(&buf_[offset + 1], (uint8_t)(length + 2)) != crcRx) {
    return PARSE_BAD;
  }
  if (out != 0) {
    out->type = buf_[offset + 2];
    out->length = length;
    for (uint8_t i = 0; i < length; i++) {
      out->payload[i] = buf_[offset + 3 + i];
    }
  }
  if (total != 0) {
    *total = want;
  }
  return PARSE_OK;
}

int16_t FrameReader::recoverFromLaterSof() const {
  for (uint8_t offset = 1; offset < len_; offset++) {
    if (buf_[offset] != ORB_SOF) {
      continue;
    }
    const Parse status = parseAt(offset, 0, 0);
    if (status == PARSE_OK) {
      return (int16_t)offset;
    }
    if (status == PARSE_INCOMPLETE) {
      return -1;  // not enough bytes to judge; wait for more
    }
  }
  return -1;
}

bool FrameReader::next(Frame* out) {
  while (true) {
    // Find the start byte.
    uint8_t start = 0;
    while (start < len_ && buf_[start] != ORB_SOF) {
      start++;
    }
    if (start >= len_) {
      if (len_ > 0) {
        resyncs++;
        len_ = 0;
      }
      return false;
    }
    if (start > 0) {
      resyncs++;
      dropFront(start);
    }

    uint8_t total = 0;
    const Parse status = parseAt(0, out, &total);
    if (status == PARSE_OK) {
      dropFront(total);
      return true;
    }
    if (status == PARSE_BAD) {
      // Bad CRC or an impossible length: drop the false start byte and rescan.
      // This is also what stops a corrupted length field stalling the reader
      // forever, waiting for bytes that are never coming.
      if (len_ >= 2 && buf_[1] <= ORB_MAX_PAYLOAD) {
        crcErrors++;
      } else {
        resyncs++;
      }
      dropFront(1);
      continue;
    }

    // Incomplete: the normal case for a frame still arriving — but also what a
    // false start byte looks like when its length field happens to be
    // plausible, with a real frame sitting right behind it. Before settling in
    // to wait, check whether a later start byte already yields a complete,
    // CRC-valid frame. Without this, whether a frame is seen depends on how the
    // UART happened to chunk the bytes.
    const int16_t recovered = recoverFromLaterSof();
    if (recovered < 0) {
      return false;
    }
    resyncs++;
    dropFront((uint8_t)recovered);
  }
}
