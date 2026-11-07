#pragma once

#include <cassert>
#include <cstddef>
#include <cstdint>
#include "esphome/core/gpio.h"

namespace redlink_test {
void transfer_status(uint8_t *data, size_t size);
}
namespace esphome::spi {
constexpr int BIT_ORDER_MSB_FIRST = 0;
constexpr int CLOCK_POLARITY_LOW = 0;
constexpr int CLOCK_PHASE_LEADING = 0;
constexpr int DATA_RATE_1MHZ = 1000000;

template<int, int, int, int> class SPIDevice {
 public:
  void spi_setup() {}
  bool spi_is_ready() const { return true; }
  void enable() {
    assert(!selected_);
    selected_ = true;
  }
  void disable() {
    assert(selected_);
    selected_ = false;
  }
  void transfer_array(uint8_t *data, size_t size) {
    assert(selected_);
    redlink_test::transfer_status(data, size);
  }
  // A wait-state test must never issue a strobe, FIFO access, or register write.
  uint8_t transfer_byte(uint8_t) {
    assert(false);
    return 0;
  }
  void read_array(uint8_t *, size_t) { assert(false); }
  void write_array(const uint8_t *, size_t) { assert(false); }

 protected:
  void *parent_{nullptr};
  InternalGPIOPin *cs_{nullptr};

 private:
  bool selected_{false};
};
}  // namespace esphome::spi
