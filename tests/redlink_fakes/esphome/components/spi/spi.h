#pragma once

#include <cassert>
#include <cstddef>
#include <cstdint>
#include "esphome/core/gpio.h"

namespace redlink_test {
void transfer_status(uint8_t *data, size_t size);
uint8_t transfer_command(uint8_t command);
void write_bytes(const uint8_t *data, size_t size);
void read_bytes(uint8_t *data, size_t size);
}  // namespace redlink_test
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
  uint8_t transfer_byte(uint8_t command) {
    assert(selected_);
    return redlink_test::transfer_command(command);
  }
  void read_array(uint8_t *data, size_t size) {
    assert(selected_);
    redlink_test::read_bytes(data, size);
  }
  void write_array(const uint8_t *data, size_t size) {
    assert(selected_);
    redlink_test::write_bytes(data, size);
  }

 protected:
  void *parent_{nullptr};
  InternalGPIOPin *cs_{nullptr};

 private:
  bool selected_{false};
};
}  // namespace esphome::spi
