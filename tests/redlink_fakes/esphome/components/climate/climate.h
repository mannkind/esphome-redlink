#pragma once

#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <optional>
#include <string>
#include <vector>

#define LOG_CLIMATE(...) ::esphome::test_log(__VA_ARGS__)

namespace esphome {
enum class TemperatureUnit { FAHRENHEIT };
namespace climate {
enum ClimateMode { CLIMATE_MODE_OFF, CLIMATE_MODE_HEAT, CLIMATE_MODE_COOL, CLIMATE_MODE_HEAT_COOL };
enum ClimateFanMode { CLIMATE_FAN_AUTO, CLIMATE_FAN_ON };
enum ClimatePreset { CLIMATE_PRESET_NONE };
constexpr uint32_t CLIMATE_SUPPORTS_CURRENT_TEMPERATURE = 1 << 0;
constexpr uint32_t CLIMATE_SUPPORTS_TWO_POINT_TARGET_TEMPERATURE = 1 << 1;
constexpr uint32_t CLIMATE_REQUIRES_TWO_POINT_TARGET_TEMPERATURE = 1 << 2;
constexpr uint32_t CLIMATE_SUPPORTS_CURRENT_HUMIDITY = 1 << 3;
constexpr uint32_t CLIMATE_SUPPORTS_TARGET_HUMIDITY = 1 << 4;
constexpr uint32_t CLIMATE_SUPPORTS_ACTION = 1 << 5;
class ClimateTraits {
 public:
  uint32_t get_feature_flags() const { return feature_flags_; }
  void add_feature_flags(uint32_t flags) { feature_flags_ |= flags; }
  bool has_feature_flags(uint32_t flags) const { return feature_flags_ & flags; }
  void set_supported_modes(std::initializer_list<ClimateMode>) {}
  void set_supported_fan_modes(std::initializer_list<ClimateFanMode>) {}
  void set_temperature_unit(TemperatureUnit unit) { temperature_unit_ = unit; }
  TemperatureUnit get_temperature_unit() const { return temperature_unit_; }
  void set_visual_min_temperature(float) {}
  void set_visual_max_temperature(float) {}
  void set_visual_temperature_step(float) {}

 private:
  uint32_t feature_flags_{0};
  TemperatureUnit temperature_unit_{TemperatureUnit::FAHRENHEIT};
};
class Climate;
class ClimateCall {
 public:
  explicit ClimateCall(Climate *) {}
  ClimateCall &set_target_temperature_low(float value) {
    low_ = value;
    return *this;
  }
  ClimateCall &set_target_temperature_high(float value) {
    high_ = value;
    return *this;
  }
  ClimateCall &set_mode(ClimateMode value) {
    mode_ = value;
    return *this;
  }
  ClimateCall &set_fan_mode(ClimateFanMode value) {
    fan_ = value;
    return *this;
  }
  ClimateCall &set_fan_mode(const char *value) {
    custom_fan_ = value;
    return *this;
  }
  ClimateCall &set_preset(const char *value) {
    custom_preset_ = value;
    return *this;
  }
  const std::optional<float> &get_target_temperature_low() const { return low_; }
  const std::optional<float> &get_target_temperature_high() const { return high_; }
  const std::optional<ClimateMode> &get_mode() const { return mode_; }
  const std::optional<ClimateFanMode> &get_fan_mode() const { return fan_; }
  const std::string &get_custom_fan_mode() const { return custom_fan_; }
  const std::string &get_custom_preset() const { return custom_preset_; }
  bool has_custom_fan_mode() const { return !custom_fan_.empty(); }
  bool has_custom_preset() const { return !custom_preset_.empty(); }

 private:
  std::optional<float> low_, high_;
  std::optional<ClimateMode> mode_;
  std::optional<ClimateFanMode> fan_;
  std::string custom_fan_, custom_preset_;
};
class Climate {
 public:
  virtual ~Climate() = default;
  struct Publication {
    float low, high;
    ClimateMode mode;
    float temperature, humidity;
  };
  std::vector<Publication> publications;
  float target_temperature_low{NAN}, target_temperature_high{NAN}, current_temperature{NAN}, current_humidity{NAN};
  ClimateMode mode{CLIMATE_MODE_OFF};
  std::optional<ClimateFanMode> fan_mode;
  std::optional<ClimatePreset> preset;
  ClimateCall make_call() { return ClimateCall(this); }
  void publish_state() {
    publications.push_back(
        {target_temperature_low, target_temperature_high, mode, current_temperature, current_humidity});
  }
  void set_supported_custom_fan_modes(std::initializer_list<const char *>) {}
  void set_supported_custom_presets(std::initializer_list<const char *>) {}
  const std::string &get_custom_fan_mode() const { return custom_fan_; }
  const std::string &get_custom_preset() const { return custom_preset_; }

 protected:
  virtual ClimateTraits traits() = 0;
  virtual void control(const ClimateCall &) = 0;
  void set_fan_mode_(ClimateFanMode value) {
    fan_mode = value;
    custom_fan_.clear();
  }
  void set_custom_fan_mode_(const char *value) {
    custom_fan_ = value;
    fan_mode.reset();
  }
  void set_custom_preset_(const char *value) {
    custom_preset_ = value;
    preset.reset();
  }
  void clear_custom_fan_mode_() { custom_fan_.clear(); }
  void clear_custom_preset_() { custom_preset_.clear(); }

 private:
  std::string custom_fan_, custom_preset_;
};
}  // namespace climate
}  // namespace esphome
