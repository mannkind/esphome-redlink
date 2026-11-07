#pragma once

#include <cstdint>

namespace esphome {
class ESPPreferenceObject {
 public:
  template<typename T> bool load(T *) { return false; }
  template<typename T> bool save(const T *) { return true; }
};
class ESPPreferences {
 public:
  template<typename T> ESPPreferenceObject make_preference(uint32_t) { return {}; }
  bool sync() { return true; }
};
inline ESPPreferences *global_preferences = nullptr;
}  // namespace esphome
