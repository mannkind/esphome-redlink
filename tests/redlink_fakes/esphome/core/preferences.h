#pragma once

#include <cstdint>
#include <cstring>
#include <map>
#include <vector>

namespace esphome {
inline std::map<uint32_t, std::vector<uint8_t>> preference_records;
class ESPPreferenceObject {
 public:
  explicit ESPPreferenceObject(uint32_t key = 0) : key_(key) {}
  template<typename T> bool load(T *value) {
    const auto found = preference_records.find(key_);
    if (found == preference_records.end() || found->second.size() != sizeof(T))
      return false;
    std::memcpy(value, found->second.data(), sizeof(T));
    return true;
  }
  template<typename T> bool save(const T *value) {
    const auto *bytes = reinterpret_cast<const uint8_t *>(value);
    preference_records[key_] = std::vector<uint8_t>(bytes, bytes + sizeof(T));
    return true;
  }

 private:
  uint32_t key_;
};
class ESPPreferences {
 public:
  template<typename T> ESPPreferenceObject make_preference(uint32_t key) { return ESPPreferenceObject(key); }
  bool sync() { return true; }
};
inline ESPPreferences *global_preferences = nullptr;
}  // namespace esphome
