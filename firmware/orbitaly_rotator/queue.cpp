#include "queue.h"

SegmentQueue::SegmentQueue()
    : head_(0), count_(0), recentCount_(0), recentNext_(0) {
  for (uint8_t i = 0; i < ORB_RECENT_DEPTH; i++) {
    recent_[i] = ORB_SEQ_NONE;
  }
}

bool SegmentQueue::push(const QueuedSegment& segment) {
  if (count_ >= ORB_QUEUE_DEPTH) {
    return false;
  }
  const uint8_t slot = (uint8_t)((head_ + count_) % ORB_QUEUE_DEPTH);
  slots_[slot] = segment;
  count_++;
  return true;
}

const QueuedSegment* SegmentQueue::peek() const {
  if (count_ == 0) {
    return 0;
  }
  return &slots_[head_];
}

void SegmentQueue::pop() {
  if (count_ == 0) {
    return;
  }
  remember(slots_[head_].seq);
  head_ = (uint8_t)((head_ + 1) % ORB_QUEUE_DEPTH);
  count_--;
}

void SegmentQueue::clear() {
  // Sequence numbers of dropped segments are remembered too. The host will
  // never re-send them — it replans from the abort — but if it did, executing
  // work it has already written off would move the antenna without permission.
  while (count_ > 0) {
    pop();
  }
}

bool SegmentQueue::isDuplicate(uint8_t seq) const {
  for (uint8_t i = 0; i < count_; i++) {
    const uint8_t slot = (uint8_t)((head_ + i) % ORB_QUEUE_DEPTH);
    if (slots_[slot].seq == seq) {
      return true;
    }
  }
  for (uint8_t i = 0; i < recentCount_; i++) {
    if (recent_[i] == seq) {
      return true;
    }
  }
  return false;
}

void SegmentQueue::remember(uint8_t seq) {
  recent_[recentNext_] = seq;
  recentNext_ = (uint8_t)((recentNext_ + 1) % ORB_RECENT_DEPTH);
  if (recentCount_ < ORB_RECENT_DEPTH) {
    recentCount_++;
  }
}
