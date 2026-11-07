#pragma once

#include "hal.h"

namespace esphome {
// Host tests are single-threaded; no GPIO ISR can run here.
class InterruptLock {
 public:
  InterruptLock() {}
  ~InterruptLock() {}
};
class HighFrequencyLoopRequester {
 public:
  void start() {}
};
inline void delay_microseconds_safe(uint32_t duration) { delayMicroseconds(duration); }
}  // namespace esphome
