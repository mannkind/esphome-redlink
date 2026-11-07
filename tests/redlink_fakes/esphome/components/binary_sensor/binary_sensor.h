#pragma once

#include <vector>

namespace esphome::binary_sensor {
class BinarySensor {
 public:
  bool state{false};
  std::vector<bool> publications;
  void publish_state(bool value) {
    state = value;
    has_state_ = true;
    publications.push_back(value);
  }
  bool has_state() const { return has_state_; }

 private:
  bool has_state_{false};
};
}  // namespace esphome::binary_sensor
