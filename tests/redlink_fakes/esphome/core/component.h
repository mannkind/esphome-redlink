#pragma once

namespace esphome {
namespace setup_priority {
constexpr float HARDWARE = 800.0f;
}
class Component {
 public:
  virtual ~Component() = default;
  virtual void setup() {}
  virtual void loop() {}
  virtual void dump_config() {}
  virtual float get_setup_priority() const { return 0; }
  bool is_failed() const { return failed_; }
  void mark_failed() { failed_ = true; }

 private:
  bool failed_{false};
};
}  // namespace esphome
