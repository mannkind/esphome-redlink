#pragma once

#include "radio.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/select/select.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"

namespace esphome::redlink {

class RedLinkSystemModeSelect;

class RedLinkClimate : public RedLinkRadio, public climate::Climate {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  void set_network_id(uint16_t value) { network_ = value; }
  void set_gateway_id(uint16_t value) { gateway_ = value; }
  void set_tx_advance_us(int32_t value) { tx_advance_us_ = value; }
  void set_result_sensor(text_sensor::TextSensor *value) { result_ = value; }
  void set_acknowledged_sensor(binary_sensor::BinarySensor *value) { acknowledged_ = value; }
  void set_verified_sensor(binary_sensor::BinarySensor *value) { verified_ = value; }
  void set_clock_locked_sensor(binary_sensor::BinarySensor *value) { clock_locked_ = value; }
  void set_command_pending_sensor(binary_sensor::BinarySensor *value) { command_pending_ = value; }
  void set_emergency_heat_sensor(binary_sensor::BinarySensor *value) { emergency_heat_ = value; }
  void set_system_mode_select(RedLinkSystemModeSelect *value) { system_mode_ = value; }
  void set_pairing_status_sensor(text_sensor::TextSensor *value) { pairing_status_ = value; }
  void set_discovery_status_sensor(text_sensor::TextSensor *value) { discovery_status_ = value; }
  void rediscover_channels();
  void set_heat_hold_until_sensor(text_sensor::TextSensor *value) { heat_hold_until_ = value; }
  void set_cool_hold_until_sensor(text_sensor::TextSensor *value) { cool_hold_until_ = value; }
  void set_paired_sensor(binary_sensor::BinarySensor *value) { paired_ = value; }
  void pair_gateway();
  void complete_gateway_association();
  void set_transmit_enabled(bool enabled);
  void refresh();
  void temporary_hold_until(int hour, int minute);
  void set_system_mode(int mode);

 protected:
  climate::ClimateTraits traits() override;
  void control(const climate::ClimateCall &call) override;
  bool start_(const Control &control);
  void request_control_(const Control &control);
  bool valid_requested_targets_(const Control &control) const;
  void begin_exchange_(const Frame &frame);
  void schedule_();
  void received_(const Frame &frame, uint32_t sync_us);
  void finish_window_();
  void finish_(const char *result, bool verified);
  void publish_status_(const Status &status);
  void publish_climate_();
  void apply_requested_(const Control &control);
  bool command_pending_awaiting_() const;
  void on_radio_failure_() override;
  void report_(const char *message);
  bool fresh_clock_() const;
  void listen_for_clock_();
  bool track_clock_(uint8_t slot, uint32_t counter, uint32_t phase, ClockListenWindow next, uint32_t sync_us);
  void report_discovery_();
  bool listen_slot_(uint8_t slot);
  uint8_t scan_candidate_(uint8_t start) const;
  void report_pairing_(const char *message);
  void report_pairing_stage_();
  bool save_pairing_(bool verified);
  void registry_verified_();
  bool tx_allowed_() const;

  enum class Stage { IDLE, BEFORE, WRITE, VERIFY, PAIRING, PAIR_STATUS, CHECK_REGISTRY };
  enum class Exchange { CLOCK, SLOT, TX, RX, SETTLE };
  Stage stage_{Stage::IDLE};
  Exchange exchange_{Exchange::CLOCK};
  uint16_t network_{0}, gateway_{0};  // Required installation-specific IDs supplied by codegen.
  bool have_clock_{false}, clock_published_{false};
  bool saw_ack_{false}, all_write_acks_{true}, sent_any_{false};
  bool paired_state_{false}, pairing_saved_{false};
  bool registry_confirmed_{false};
  bool complete_association_{false};
  uint8_t attempts_{0};
  uint32_t clock_sync_us_{0}, clock_phase_ticks_{0}, clock_received_ms_{0};
  uint32_t scan_changed_ms_{0}, scan_round_{0}, scan_window_us_{0};
  std::array<uint8_t, HOP_COUNT> scan_trials_{};
  uint32_t last_clock_counter_{0};
  bool have_clock_counter_{false}, map_saved_{false};
  uint8_t clock_slot_{0}, clock_candidate_{0};
  bool clock_epoch_pending_{false};
  ESPPreferenceObject hop_preference_;
  ClockListenWindow clock_listen_{};
  uint32_t transaction_started_ms_{0}, settle_started_ms_{0}, clock_wait_started_ms_{0};
  uint32_t due_us_{0}, last_tx_us_{0}, tx_started_ms_{0}, window_started_ms_{0};
  uint32_t last_refresh_ms_{0}, retry_after_ms_{0};
  int32_t tx_advance_us_{0};
  Control requested_{};
  Control queued_control_{};
  bool have_queued_control_{false};
  // Only directed, actual readback populates confirmed_status_; requested controls
  // are a presentation overlay and expected_ is exclusively a verification target.
  Status received_status_{}, confirmed_status_{}, expected_{};
  Frame outgoing_{};
  std::array<Frame, 2> writes_{};
  size_t write_count_{0}, write_index_{0};
  HighFrequencyLoopRequester high_frequency_;
  GatewayEnrollment enrollment_;
  struct PairingMarker {
    uint32_t magic;
    uint16_t network, gateway;
  };
  ESPPreferenceObject pairing_preference_;
  text_sensor::TextSensor *result_{nullptr};
  binary_sensor::BinarySensor *acknowledged_{nullptr}, *verified_{nullptr}, *clock_locked_{nullptr};
  binary_sensor::BinarySensor *command_pending_{nullptr};
  binary_sensor::BinarySensor *emergency_heat_{nullptr};
  RedLinkSystemModeSelect *system_mode_{nullptr};
  text_sensor::TextSensor *pairing_status_{nullptr};
  text_sensor::TextSensor *discovery_status_{nullptr};
  text_sensor::TextSensor *heat_hold_until_{nullptr}, *cool_hold_until_{nullptr};
  binary_sensor::BinarySensor *paired_{nullptr};
};

class RedLinkSystemModeSelect : public select::Select {
 public:
  explicit RedLinkSystemModeSelect(RedLinkClimate *parent) : parent_(parent) {}
  void publish_mode(uint8_t mode);

 protected:
  void control(size_t index) override;
  RedLinkClimate *parent_;
};

}  // namespace esphome::redlink
