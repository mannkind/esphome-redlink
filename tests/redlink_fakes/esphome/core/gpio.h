#pragma once

#define IRAM_ATTR

namespace esphome {
namespace gpio {
constexpr int FLAG_INPUT = 1;
constexpr int INTERRUPT_RISING_EDGE = 1;
}  // namespace gpio
class InternalGPIOPin {
 public:
  bool high{false};
  bool digital_read() const { return high; }
  bool is_internal() const { return true; }
  bool is_inverted() const { return false; }
  void setup() {}
  void pin_mode(int) {}
  template<typename T> void attach_interrupt(void (*)(T *), T *, int) {}
};
}  // namespace esphome
