#pragma once

#include <cstddef>
#include <string>
#include <vector>

#define LOG_SELECT(...)

namespace esphome::select {
class SelectTraits {
 public:
  void set_options_static(const char *const *options, size_t count) {
    options_ = options;
    size_ = count;
  }
  const char *const *options_{nullptr};
  size_t size_{0};
};

class Select {
 public:
  SelectTraits traits;
  std::vector<size_t> publications;
  void publish_state(size_t index) {
    if (index >= traits.size_)
      return;
    active_index_ = index;
    has_state_ = true;
    publications.push_back(index);
  }
  bool has_state() const { return has_state_; }
  std::string current_option() const { return has_state_ ? traits.options_[active_index_] : ""; }

 protected:
  virtual void control(size_t index) = 0;
  size_t active_index_{0};
  bool has_state_{false};
};
}  // namespace esphome::select
