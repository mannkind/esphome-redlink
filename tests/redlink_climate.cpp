// Actual controller + protocol, with only ESPHome APIs and the radio transport faked.
// No GPIO, SPI, device, network, or RF activity is possible in this executable.
#include "redlink.h"
#include "esphome/core/hal.h"

#include <algorithm>
#include "check.h"
#include <cmath>
#include <cstdio>
#include <cstring>

using namespace esphome;
using namespace esphome::redlink;
namespace {
uint64_t now_us = 1000000;
unsigned transport_sends = 0;
}  // namespace
namespace esphome {
uint32_t micros() { return now_us; }
uint32_t millis() { return now_us / 1000; }
void delayMicroseconds(uint32_t duration) { now_us += duration; }
namespace redlink {
void RedLinkRadio::setup() {}
void RedLinkRadio::dump_config() {}
bool RedLinkRadio::read_frame(Frame &, uint32_t &) { return false; }
bool RedLinkRadio::send_frame(const Frame &) {
  ++transport_sends;
  tx_succeeded_ = true;
  return true;
}
bool RedLinkRadio::tx_finished() { return true; }
void RedLinkRadio::begin_rx() {}
bool RedLinkRadio::listen_candidate_(uint8_t rank) {
  if (rank >= HOP_COUNT)
    return false;
  receive_candidate_ = rank;
  return true;
}
}  // namespace redlink
}  // namespace esphome

static Status captured_status(const char *hex) {
  Frame frame;
  frame.size = std::strlen(hex) / 2;
  for (size_t i = 0; i < frame.size; ++i) {
    unsigned value;
    CHECK(std::sscanf(hex + 2 * i, "%2x", &value) == 1);
    frame.data[i] = value;
  }
  Status status;
  CHECK(collect_status(frame, status) && actual_status(status));
  return status;
}

static Status emergency_actual() {
  // User-labeled Emergency Heat, telemetry-mode-emergency68-20261107T014510Z.
  return captured_status("3428ffffc27a389e0000f01e01ff005a8046444a2e005cdc284e3c63464af802802f844600a901070020"
                         "f00922c0004500a001000fd962");
}

static Status actual(int heat = 70, int cool = 74, int hold = 2) {
  // Captured actual blocks, not write blocks relabeled by the production code.
  Status status = captured_status("3428ffffd6e0389e0000f01e01ff005a8046464a56005cdc284e3c63464afc02802f8446c8a901070020"
                                  "f00922c0004500a001000f0646");
  status.thermostat[7] = heat;
  status.thermostat[8] = cool;
  status.thermostat[9] = (status.thermostat[9] & ~0x78) | (hold << 3) | (hold << 5);
  CHECK(actual_status(status));
  return status;
}

static Status odd_room_actual() {
  Status status = actual();
  // Captured thermostat block: room 71 F, heat target 70 F, cool target 74 F, RH 99%.
  status.thermostat = {0x1e, 0x01, 0xff, 0x00, 0x5a, 0x80, 0x47, 0x46, 0x4a, 0x06, 0x00, 0x5c, 0xdc, 0x28, 0x4e, 0x3c,
                       0x63, 0x46, 0x4a, 0xfc, 0x02, 0x80, 0x63, 0x84, 0x47, 0xc8, 0xa9, 0x01, 0x07, 0x00, 0x20};
  return status;
}

static Frame reply(const Status &s) {
  Frame f;
  f.size = 55;
  const uint8_t header[] = {52, 0x28, 0xff, 0xff, 0xb1, 0x4f, 0x38, 0x9e, 0, 0, 0xf0};
  std::copy(std::begin(header), std::end(header), f.data.begin());
  std::copy(s.thermostat.begin(), s.thermostat.end(), f.data.begin() + 11);
  f.data[42] = 0xf0;
  std::copy(s.fan.begin(), s.fan.end(), f.data.begin() + 43);
  const uint16_t crc = crc16(f.data.data(), f.size - 2);
  f.data[53] = crc >> 8;
  f.data[54] = crc;
  CHECK(peer_reply(f, 0x389e, 0xb14f));
  return f;
}

class TestSystemModeSelect : public RedLinkSystemModeSelect {
 public:
  using RedLinkSystemModeSelect::RedLinkSystemModeSelect;
  using RedLinkSystemModeSelect::control;
};

class TestClimate : public RedLinkClimate {
 public:
  using RedLinkClimate::Stage;
  using RedLinkClimate::Exchange;
  using RedLinkClimate::control;
  using RedLinkClimate::traits;
  using RedLinkClimate::request_control_;
  using RedLinkClimate::publish_status_;
  using RedLinkClimate::finish_;
  using RedLinkClimate::finish_window_;
  using RedLinkClimate::received_;
  using RedLinkClimate::stage_;
  using RedLinkClimate::exchange_;
  using RedLinkClimate::requested_;
  using RedLinkClimate::queued_control_;
  using RedLinkClimate::have_queued_control_;
  using RedLinkClimate::confirmed_status_;
  using RedLinkClimate::received_status_;
  using RedLinkClimate::expected_;
  using RedLinkClimate::last_tx_us_;
  using RedLinkClimate::write_count_;
  using RedLinkClimate::have_clock_;
  using RedLinkClimate::paired_state_;
  using RedLinkClimate::hop_;
  using RedLinkClimate::schedule_;
  using RedLinkClimate::listen_for_clock_;
  using RedLinkClimate::receive_candidate_;
  using RedLinkClimate::clock_listen_;
  using RedLinkClimate::clock_received_ms_;
  using RedLinkClimate::tx_allowed_;
  using RedLinkClimate::fresh_clock_;
  using RedLinkClimate::have_clock_counter_;
  binary_sensor::BinarySensor pending, verified, acknowledged, locked, emergency;
  text_sensor::TextSensor result;
  TestSystemModeSelect system_mode{this};

  explicit TestClimate(bool seed_map = true) {
    // Match the captured test frames explicitly; production IDs have no defaults.
    set_network_id(0x389e);
    set_gateway_id(0xb14f);
    static constexpr const char *OPTIONS[] = {"Auto", "Heat", "Cool", "Off", "Emergency Heat"};
    system_mode.traits.set_options_static(OPTIONS, 5);
    set_system_mode_select(&system_mode);
    set_command_pending_sensor(&pending);
    set_verified_sensor(&verified);
    set_acknowledged_sensor(&acknowledged);
    set_clock_locked_sensor(&locked);
    set_emergency_heat_sensor(&emergency);
    set_result_sensor(&result);
    setup();
    // Explicit synthetic test permutation, never a production default.
    if (seed_map) {
      for (uint8_t slot = 0; slot < HOP_COUNT; ++slot) {
        CHECK(hop_.observe(slot, 49 - slot, 1000 + slot));
        CHECK(hop_.observe(slot, 49 - slot, 2000 + slot));
      }
    }
    receive_only_ = false;
    transmit_enabled_ = true;
    paired_state_ = true;
    have_clock_ = clock_published_ = have_clock_counter_ = seed_map;
    clock_received_ms_ = millis();
    clock_listen_.sync_us = micros() + 100000;
    locked.publish_state(true);
  }
  void read_status(const Status &s) {
    exchange_ = Exchange::RX;
    attempts_ = 1;
    last_tx_us_ = micros() - 1000;
    received_status_ = Status{};
    received_(reply(s), micros() - 500);
    finish_window_();
  }
  void writes_acknowledged() {
    while (stage_ == Stage::WRITE) {
      attempts_ = 1;
      saw_ack_ = true;
      finish_window_();
    }
    CHECK(stage_ == Stage::VERIFY);
  }
  void fail_radio() {
    mark_failed();
    on_radio_failure_();
  }
};

static Control targets(int heat, int cool = -1) {
  Control c;
  c.heat = heat;
  c.cool = cool;
  return c;
}

static void measured_climate_traits_and_initial_unknown() {
  TestClimate climate;
  const auto traits = climate.traits();
  CHECK(traits.has_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE));
  CHECK(traits.has_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_HUMIDITY));
  CHECK(traits.has_feature_flags(climate::CLIMATE_SUPPORTS_TWO_POINT_TARGET_TEMPERATURE));
  CHECK(traits.has_feature_flags(climate::CLIMATE_REQUIRES_TWO_POINT_TARGET_TEMPERATURE));
  CHECK(!traits.has_feature_flags(climate::CLIMATE_SUPPORTS_ACTION | climate::CLIMATE_SUPPORTS_TARGET_HUMIDITY));
  CHECK(traits.get_temperature_unit() == TemperatureUnit::FAHRENHEIT);
  CHECK(std::isnan(climate.current_temperature) && std::isnan(climate.current_humidity));
  climate.request_control_(targets(72));
  Status echo = odd_room_actual();
  echo.thermostat[4] = 0x9a;
  climate.read_status(echo);
  CHECK(climate.stage_ == TestClimate::Stage::BEFORE && climate.pending.state);
  CHECK(std::isnan(climate.publications.back().temperature) && std::isnan(climate.publications.back().humidity));
  CHECK(climate.target_temperature_low == 72);
  climate.finish_("no actual status", false);
  CHECK(std::isnan(climate.current_temperature) && std::isnan(climate.current_humidity));
  climate.refresh();
  climate.read_status(odd_room_actual());
  CHECK(climate.current_temperature == 71 && climate.target_temperature_low == 70);
  CHECK(climate.publications.back().temperature == 71 && climate.publications.back().humidity == 99);
}

static void measurements_are_independently_guarded() {
  TestClimate climate;
  climate.publish_status_(odd_room_actual());
  Status status = actual();
  status.thermostat[6] = 71;  // Duplicates disagree; humidity remains usable.
  status.thermostat[22] = 49;
  climate.publish_status_(status);
  CHECK(std::isnan(climate.current_temperature) && climate.current_humidity == 49);
  CHECK(climate.target_temperature_low == 70);
  for (uint8_t invalid : {uint8_t{128}, uint8_t{255}}) {
    status.thermostat[6] = status.thermostat[24] = invalid;
    climate.publish_status_(status);
    CHECK(std::isnan(climate.current_temperature) && climate.current_humidity == 49);
  }
  status = odd_room_actual();
  for (uint8_t invalid : {uint8_t{101}, uint8_t{255}}) {
    status.thermostat[22] = invalid;
    climate.publish_status_(status);
    CHECK(climate.current_temperature == 71 && std::isnan(climate.current_humidity));
  }
  for (uint8_t boundary : {uint8_t{0}, uint8_t{100}}) {
    status.thermostat[22] = boundary;
    climate.publish_status_(status);
    CHECK(climate.current_temperature == 71 && climate.current_humidity == boundary);
  }
  // Invalid measurements must not prevent fresh control context or verification.
  status = actual();
  status.thermostat[6] = status.thermostat[24] = 128;
  status.thermostat[22] = 101;
  climate.request_control_(targets(72));
  climate.read_status(status);
  CHECK(climate.stage_ == TestClimate::Stage::WRITE && climate.target_temperature_low == 72);
  CHECK(std::isnan(climate.current_temperature) && std::isnan(climate.current_humidity));
  climate.writes_acknowledged();
  status.thermostat[7] = 72;
  climate.read_status(status);
  CHECK(climate.verified.state && !climate.pending.state);
  CHECK(std::isnan(climate.publications.back().temperature) && std::isnan(climate.publications.back().humidity));
}

static void measurements_during_optimism_verification_and_rollback() {
  TestClimate climate;
  climate.publish_status_(actual());
  climate.request_control_(targets(72));
  Status warmed = actual();
  warmed.thermostat[6] = warmed.thermostat[24] = 71;
  warmed.thermostat[22] = 99;
  climate.read_status(warmed);
  CHECK(climate.stage_ == TestClimate::Stage::WRITE && climate.target_temperature_low == 72);
  CHECK(climate.publications.back().temperature == 71 && climate.publications.back().humidity == 99);
  climate.request_control_(targets(-1, 75));  // Queue overlays only the controls.
  CHECK(climate.current_temperature == 71 && climate.current_humidity == 99);
  climate.writes_acknowledged();
  Status echo = actual(72);
  echo.thermostat[4] = 0x9a;
  echo.thermostat[6] = echo.thermostat[24] = 80;
  echo.thermostat[22] = 10;
  climate.read_status(echo);
  CHECK(climate.stage_ == TestClimate::Stage::VERIFY && !climate.verified.state);
  CHECK(climate.current_temperature == 71 && climate.current_humidity == 99);
  warmed.thermostat[6] = warmed.thermostat[24] = 72;
  warmed.thermostat[22] = 74;
  climate.read_status(warmed);  // Target still mismatches, but measurements are actual.
  CHECK(climate.stage_ == TestClimate::Stage::VERIFY && !climate.verified.state);
  CHECK(climate.target_temperature_low == 72 && climate.target_temperature_high == 75);
  CHECK(climate.publications.back().temperature == 72 && climate.publications.back().humidity == 74);
  climate.finish_("readback timeout", false);
  CHECK(!climate.pending.state && !climate.verified.state);
  CHECK(climate.target_temperature_low == 70 && climate.target_temperature_high == 74);
  CHECK(climate.publications.back().temperature == 72 && climate.publications.back().humidity == 74);

  climate.request_control_(targets(72));
  climate.read_status(warmed);
  climate.writes_acknowledged();
  Status applied = actual(72);
  applied.thermostat[6] = applied.thermostat[24] = 71;
  applied.thermostat[22] = 49;
  climate.read_status(applied);  // Telemetry changes cannot invalidate matched controls.
  CHECK(climate.verified.state && !climate.pending.state);
  CHECK(climate.target_temperature_low == 72 && climate.current_temperature == 71);
  CHECK(climate.publications.back().humidity == 49);
}

static void immediate_optimism_and_rollback() {
  TestClimate unknown;
  unknown.request_control_(targets(71));
  CHECK(unknown.pending.state && !unknown.verified.state);
  CHECK(unknown.target_temperature_low == 71 && std::isnan(unknown.target_temperature_high));
  CHECK(!unknown.confirmed_status_.complete());
  unknown.finish_("no clock", false);
  CHECK(!unknown.pending.state && !unknown.verified.state);
  CHECK(std::isnan(unknown.target_temperature_low) && std::isnan(unknown.target_temperature_high));

  TestClimate climate;
  const Status baseline = actual();
  climate.publish_status_(baseline);
  climate.request_control_(targets(71));
  CHECK(climate.target_temperature_low == 71 && climate.target_temperature_high == 74);
  CHECK(climate.confirmed_status_.thermostat == baseline.thermostat);
  const size_t publications = climate.publications.size();
  climate.read_status(baseline);
  CHECK(climate.stage_ == TestClimate::Stage::WRITE && climate.expected_.thermostat[7] == 71);
  CHECK(climate.confirmed_status_.thermostat[7] == 70 && !climate.verified.state);
  for (size_t i = publications; i < climate.publications.size(); ++i)
    CHECK(climate.publications[i].low == 71);  // Fresh pre-write state must not bounce the UI.
  climate.finish_("write failed", false);
  CHECK(climate.target_temperature_low == 70 && !climate.pending.state);
}

static void merged_and_queued_controls() {
  TestClimate climate;
  climate.publish_status_(actual());
  climate.request_control_(targets(71));
  Control mode_and_fan;
  mode_and_fan.mode = 1;
  mode_and_fan.fan = 0;
  climate.request_control_(mode_and_fan);
  CHECK(climate.requested_.heat == 71 && climate.mode == climate::CLIMATE_MODE_HEAT);
  CHECK(climate.fan_mode == climate::CLIMATE_FAN_AUTO);
  climate.read_status(actual());
  CHECK(climate.write_count_ == 2);
  const Status first_expected = climate.expected_;
  climate.request_control_(targets(72));
  climate.request_control_(targets(-1, 75));
  CHECK(climate.requested_.heat == 71 && climate.queued_control_.heat == 72);
  CHECK(climate.expected_.thermostat == first_expected.thermostat);
  CHECK(climate.target_temperature_low == 72 && climate.target_temperature_high == 75);
  climate.writes_acknowledged();
  CHECK(!climate.acknowledged.state && !climate.verified.state && climate.pending.state);
  Status first_actual = actual(71);
  first_actual.thermostat[19] = (first_actual.thermostat[19] & ~7) | 1;
  first_actual.fan[9] = (first_actual.fan[9] & ~3);
  const size_t pending_publications = climate.pending.publications.size();
  climate.read_status(first_actual);
  CHECK(climate.stage_ == TestClimate::Stage::IDLE && climate.have_queued_control_);
  CHECK(climate.confirmed_status_.thermostat[7] == 71);
  CHECK(climate.target_temperature_low == 72 && climate.pending.state && !climate.verified.state);
  climate.loop();  // Consume the queue without a false pending or climate-state pulse.
  CHECK(climate.stage_ == TestClimate::Stage::BEFORE && !climate.have_queued_control_);
  for (size_t i = pending_publications; i < climate.pending.publications.size(); ++i)
    CHECK(climate.pending.publications[i]);
  climate.read_status(first_actual);
  climate.writes_acknowledged();
  Status second_actual = first_actual;
  second_actual.thermostat[7] = 72;
  second_actual.thermostat[8] = 75;
  climate.read_status(second_actual);
  CHECK(!climate.pending.state && climate.verified.state && !climate.have_queued_control_);
  CHECK(climate.confirmed_status_.thermostat[7] == 72 && climate.target_temperature_high == 75);
}

static void mismatched_readback_rolls_back_to_latest_actual() {
  TestClimate climate;
  climate.publish_status_(actual());
  climate.request_control_(targets(72));
  climate.read_status(actual());
  climate.writes_acknowledged();
  climate.request_control_(targets(-1, 75));
  climate.read_status(actual(69));
  CHECK(climate.stage_ == TestClimate::Stage::VERIFY && !climate.verified.state);
  CHECK(climate.confirmed_status_.thermostat[7] == 69);
  CHECK(climate.target_temperature_low == 72 && climate.target_temperature_high == 75);
  climate.finish_("readback timeout", false);
  CHECK(!climate.pending.state && !climate.have_queued_control_ && !climate.verified.state);
  CHECK(climate.target_temperature_low == 69 && climate.target_temperature_high == 74);
}

static void invalid_calls_are_atomic() {
  TestClimate climate;
  climate.publish_status_(actual());
  for (float invalid : {54.0f, 81.0f, 70.5f, NAN, INFINITY}) {
    auto call = climate.make_call();
    call.set_target_temperature_low(invalid).set_target_temperature_high(75);
    climate.control(call);
    CHECK(!climate.pending.state && climate.stage_ == TestClimate::Stage::IDLE);
    CHECK(climate.target_temperature_low == 70 && climate.target_temperature_high == 74);
  }
  for (float invalid : {54.0f, 81.0f, 74.5f, NAN, INFINITY}) {
    auto call = climate.make_call();
    call.set_target_temperature_low(71).set_target_temperature_high(invalid);
    climate.control(call);
    CHECK(!climate.pending.state && climate.target_temperature_low == 70);
  }
  auto unsupported = climate.make_call();
  unsupported.set_target_temperature_low(71).set_preset("mixed");
  climate.control(unsupported);
  CHECK(!climate.pending.state && climate.target_temperature_low == 70);
  auto conflict = climate.make_call();
  conflict.set_target_temperature_low(71).set_preset("schedule");
  climate.control(conflict);
  CHECK(!climate.pending.state && climate.stage_ == TestClimate::Stage::IDLE);
  climate.control(climate.make_call());
  CHECK(climate.stage_ == TestClimate::Stage::IDLE);
  climate.temporary_hold_until(24, 0);
  climate.temporary_hold_until(12, 10);
  CHECK(!climate.pending.state);
  climate.request_control_(targets(71));
  climate.control(unsupported);
  CHECK(climate.pending.state && climate.target_temperature_low == 71);  // Rejection is not cancellation.
}

static void schedule_merges_and_no_op() {
  TestClimate climate;
  climate.publish_status_(actual());
  climate.request_control_(targets(71));
  Control schedule;
  schedule.hold = 0;
  climate.request_control_(schedule);
  CHECK(climate.requested_.heat == -1 && climate.requested_.hold == 0);
  CHECK(climate.target_temperature_low == 70 && climate.get_custom_preset() == "schedule");
  climate.request_control_(targets(72));
  CHECK(climate.requested_.heat == 72 && climate.requested_.hold == -1);
  climate.finish_("cancel", false);
  climate.request_control_(targets(70));
  CHECK(climate.pending.state && !climate.verified.state);
  climate.read_status(actual());
  CHECK(climate.stage_ == TestClimate::Stage::IDLE && climate.write_count_ == 0);
  CHECK(!climate.pending.state && climate.verified.state);
}

static void refresh_and_pending_status() {
  TestClimate climate;
  climate.publish_status_(actual());
  climate.refresh();
  CHECK(!climate.pending.state && climate.stage_ == TestClimate::Stage::BEFORE);
  climate.read_status(actual(69));
  CHECK(!climate.pending.state && climate.verified.state && climate.target_temperature_low == 69);
  climate.request_control_(targets(71));
  climate.refresh();  // A refresh cannot replace or clear the accepted command.
  CHECK(climate.pending.state && climate.requested_.heat == 71 && climate.target_temperature_low == 71);
  Status pending = actual(72);
  pending.thermostat[4] = 0x9a;
  climate.publish_status_(pending);
  CHECK(climate.confirmed_status_.thermostat[7] == 69);
  CHECK(climate.target_temperature_low == 71 && !climate.verified.state);
  climate.finish_("cancel", false);
  climate.pair_gateway();
  CHECK(climate.stage_ == TestClimate::Stage::PAIRING && !climate.pending.state);
  climate.request_control_(targets(72));
  CHECK(!climate.pending.state && climate.target_temperature_low == 69);
}

static void transmit_disable_and_radio_failure() {
  TestClimate climate;
  climate.publish_status_(actual());
  climate.request_control_(targets(71));
  climate.read_status(actual());
  climate.request_control_(targets(72));
  climate.exchange_ = TestClimate::Exchange::TX;
  climate.set_transmit_enabled(false);
  CHECK(climate.exchange_ == TestClimate::Exchange::TX);  // Do not cut an on-air packet.
  CHECK(!climate.pending.state && !climate.have_queued_control_ && !climate.verified.state);
  CHECK(climate.target_temperature_low == 70);
  climate.request_control_(targets(72));
  CHECK(!climate.pending.state && climate.target_temperature_low == 70);
  climate.set_transmit_enabled(true);  // Old frame still on air; no new request can start.
  climate.request_control_(targets(72));
  CHECK(!climate.pending.state && climate.target_temperature_low == 70);
  climate.loop();
  climate.request_control_(targets(71));
  CHECK(climate.pending.state);
  climate.fail_radio();  // No subsequent loop: component failure may suppress it entirely.
  CHECK(climate.is_failed() && !climate.pending.state && !climate.verified.state);
  CHECK(!climate.locked.state && !climate.have_clock_ && climate.paired_state_);
  CHECK(climate.target_temperature_low == 70 && climate.confirmed_status_.thermostat[7] == 70);
}

static void stale_prewrite_reply(uint32_t query_sync) {
  TestClimate climate;
  climate.publish_status_(actual());
  climate.request_control_(targets(71));
  climate.read_status(actual());
  climate.writes_acknowledged();
  climate.exchange_ = TestClimate::Exchange::RX;
  climate.last_tx_us_ = query_sync;
  climate.received_status_ = Status{};
  // Even a matching actual frame cannot verify if its sync predates this query.
  climate.received_(reply(actual(71)), query_sync - 500);
  CHECK(!climate.received_status_.complete());
  CHECK(climate.confirmed_status_.thermostat[7] == 70 && !climate.verified.state);
  climate.received_(reply(actual(71)), query_sync + 500);
  CHECK(climate.received_status_.complete());
  climate.finish_window_();
  CHECK(climate.verified.state && !climate.pending.state && climate.confirmed_status_.thermostat[7] == 71);
}

static void emergency_heat_readback_and_optimism() {
  TestClimate climate;
  CHECK(climate.emergency.publications.empty());  // Unknown until actual readback, not false at boot.
  CHECK(!climate.system_mode.has_state() && climate.system_mode.publications.empty());
  climate.refresh();
  climate.read_status(emergency_actual());
  CHECK(climate.verified.state && !climate.pending.state);
  CHECK(climate.mode == climate::CLIMATE_MODE_HEAT && climate.emergency.state);
  CHECK(climate.system_mode.current_option() == "Emergency Heat");
  CHECK(climate.target_temperature_low == 68 && climate.get_custom_preset() == "temporary");

  // An unrelated target request retains emergency mode through preparation and verification.
  climate.request_control_(targets(69));
  CHECK(climate.emergency.state && climate.mode == climate::CLIMATE_MODE_HEAT);
  climate.read_status(emergency_actual());
  CHECK(climate.stage_ == TestClimate::Stage::WRITE && climate.expected_.thermostat[19] == 0xf8);
  climate.writes_acknowledged();
  CHECK(climate.emergency.state && !climate.verified.state);
  Status emergency = emergency_actual();
  emergency.thermostat[7] = 69;
  climate.read_status(emergency);
  CHECK(climate.verified.state && climate.emergency.state && !climate.pending.state);

  // Neither an optimistic request, an ACK, nor a failed transaction clears actual Emergency Heat.
  Control off;
  off.mode = 2;
  const size_t confirmed_publications = climate.emergency.publications.size();
  climate.request_control_(off);
  CHECK(climate.mode == climate::CLIMATE_MODE_OFF && climate.emergency.state);
  CHECK(climate.emergency.publications.size() == confirmed_publications);
  climate.read_status(emergency);
  climate.writes_acknowledged();
  CHECK(climate.emergency.state);
  climate.finish_("unverified mode change", false);
  CHECK(climate.mode == climate::CLIMATE_MODE_HEAT && climate.emergency.state);

  // Native climate HEAT explicitly leaves emergency mode even though both display base HEAT.
  auto heat = climate.make_call();
  heat.set_mode(climate::CLIMATE_MODE_HEAT);
  climate.control(heat);
  climate.read_status(emergency);
  CHECK(climate.write_count_ == 1 && climate.expected_.thermostat[19] == 0xf9);
  climate.writes_acknowledged();
  CHECK(climate.emergency.state);
  Status normal = emergency;
  normal.thermostat[19] = 0xf9;
  climate.read_status(normal);
  CHECK(climate.verified.state && !climate.emergency.state && climate.mode == climate::CLIMATE_MODE_HEAT);
  CHECK(climate.system_mode.current_option() == "Heat");

  // New actual Emergency Heat remains visible separately while an optimistic Off masks base HEAT.
  climate.request_control_(off);
  climate.read_status(emergency);
  CHECK(climate.mode == climate::CLIMATE_MODE_OFF && climate.emergency.state);
  climate.writes_acknowledged();
  normal.thermostat[19] = 0xfa;
  climate.read_status(normal);
  CHECK(climate.mode == climate::CLIMATE_MODE_OFF && !climate.emergency.state && climate.verified.state);

  climate.publish_status_(emergency);
  for (int mode : {5, 6, 7}) {
    Control rejected;
    rejected.mode = mode;
    climate.request_control_(rejected);
    CHECK(climate.stage_ == TestClimate::Stage::IDLE && !climate.pending.state);
    CHECK(climate.emergency.state && climate.mode == climate::CLIMATE_MODE_HEAT);
    Status unsupported = normal;
    unsupported.thermostat[19] = 0xf8 | mode;
    climate.publish_status_(unsupported);
    CHECK(climate.emergency.state && climate.confirmed_status_.thermostat[19] == 0xf8);
  }
  Status pending = normal;
  pending.thermostat[4] = 0x9a;
  climate.publish_status_(pending);
  CHECK(climate.emergency.state);  // Pending write echoes are not confirmed operating mode.
  CHECK(climate.system_mode.current_option() == "Emergency Heat");
}

static void system_mode_selection_and_no_op() {
  TestClimate unknown;
  CHECK(unknown.stage_ == TestClimate::Stage::IDLE && !unknown.pending.state);
  CHECK(unknown.system_mode.publications.empty() && !unknown.system_mode.has_state());
  unknown.system_mode.control(5);  // No option may alias the unknown wire-mode code 5.
  CHECK(unknown.stage_ == TestClimate::Stage::IDLE && !unknown.pending.state);
  unknown.system_mode.control(4);  // Emergency Heat before any status is available.
  CHECK(unknown.requested_.mode == 0 && unknown.pending.state && !unknown.system_mode.has_state());
  unknown.finish_("no fresh status", false);
  CHECK(!unknown.pending.state && !unknown.system_mode.has_state() && !unknown.emergency.has_state());

  // Exercise every select index against independent actual mode readback, not expected_.
  constexpr std::array<int, 5> CODES{4, 1, 3, 2, 0};
  const char *OPTIONS[] = {"Auto", "Heat", "Cool", "Off", "Emergency Heat"};
  for (size_t index = 0; index < CODES.size(); ++index) {
    TestClimate climate;
    Status before = emergency_actual();
    if (index == 4)
      before.thermostat[19] = 0xfc;
    climate.publish_status_(before);
    const auto before_option = climate.system_mode.current_option();
    const size_t publications = climate.system_mode.publications.size();
    climate.system_mode.control(index);
    CHECK(climate.stage_ == TestClimate::Stage::BEFORE && climate.requested_.mode == CODES[index]);
    CHECK(climate.pending.state && !climate.verified.state);
    CHECK(climate.system_mode.current_option() == before_option);
    CHECK(climate.system_mode.publications.size() == publications);
    CHECK(climate.get_custom_preset() == "temporary");  // Mode selection never replaces hold presets.
    climate.read_status(before);
    CHECK(climate.stage_ == TestClimate::Stage::WRITE && climate.write_count_ == 1);
    CHECK(climate.expected_.thermostat[19] == (0xf8 | CODES[index]));
    climate.writes_acknowledged();
    CHECK(climate.acknowledged.state && !climate.verified.state && climate.pending.state);
    CHECK(climate.system_mode.current_option() == before_option);
    CHECK(climate.emergency.state == ((before.thermostat[19] & 7) == 0));
    Status pending = before;
    pending.thermostat[19] = 0xf8 | CODES[index];
    pending.thermostat[4] = 0x9a;
    climate.publish_status_(pending);
    CHECK(climate.system_mode.current_option() == before_option);
    Status after = before;
    after.thermostat[19] = 0xf8 | CODES[index];
    climate.read_status(after);
    CHECK(climate.verified.state && !climate.pending.state);
    CHECK(climate.system_mode.current_option() == OPTIONS[index]);
    CHECK(climate.emergency.state == (CODES[index] == 0));
    CHECK(climate.get_custom_preset() == "temporary");
    climate.system_mode.control(index);
    CHECK(climate.pending.state);
    climate.read_status(after);
    CHECK(climate.stage_ == TestClimate::Stage::IDLE && climate.write_count_ == 0);
    CHECK(climate.verified.state && !climate.pending.state);
    CHECK(climate.system_mode.current_option() == OPTIONS[index]);
  }
}

static void system_mode_merging_queue_and_failure() {
  TestClimate climate;
  climate.publish_status_(actual());
  climate.system_mode.control(1);  // Heat superseded while the pre-read is still pending.
  climate.system_mode.control(4);
  climate.request_control_(targets(71));
  CHECK(climate.requested_.mode == 0 && climate.requested_.heat == 71);
  CHECK(climate.mode == climate::CLIMATE_MODE_HEAT && climate.system_mode.current_option() == "Auto");
  climate.read_status(actual());
  CHECK(climate.expected_.thermostat[19] == 0xf8 && climate.expected_.thermostat[7] == 71);
  climate.system_mode.control(3);  // Off is superseded in the unsent queue too.
  climate.system_mode.control(2);
  CHECK(climate.requested_.mode == 0 && climate.queued_control_.mode == 3);
  CHECK(climate.mode == climate::CLIMATE_MODE_COOL && climate.pending.state);
  climate.writes_acknowledged();
  Status emergency = actual(71);
  emergency.thermostat[19] = 0xf8;
  climate.read_status(emergency);
  CHECK(climate.stage_ == TestClimate::Stage::IDLE && climate.have_queued_control_);
  CHECK(climate.system_mode.current_option() == "Emergency Heat" && climate.emergency.state);
  CHECK(climate.mode == climate::CLIMATE_MODE_COOL && climate.pending.state && !climate.verified.state);
  const size_t pending_publications = climate.pending.publications.size();
  climate.loop();
  CHECK(climate.stage_ == TestClimate::Stage::BEFORE && climate.requested_.mode == 3);
  climate.read_status(emergency);
  climate.writes_acknowledged();
  for (size_t i = pending_publications; i < climate.pending.publications.size(); ++i)
    CHECK(climate.pending.publications[i]);
  Status cool = emergency;
  cool.thermostat[19] = 0xfb;
  climate.read_status(cool);
  CHECK(climate.system_mode.current_option() == "Cool" && !climate.emergency.state);
  CHECK(climate.verified.state && !climate.pending.state);

  climate.system_mode.control(4);
  climate.read_status(cool);
  climate.writes_acknowledged();
  climate.system_mode.control(3);
  Status heat = cool;
  heat.thermostat[19] = 0xf9;
  climate.read_status(heat);  // A real mismatched readback becomes the rollback baseline.
  CHECK(climate.stage_ == TestClimate::Stage::VERIFY && climate.system_mode.current_option() == "Heat");
  CHECK(climate.mode == climate::CLIMATE_MODE_OFF && climate.have_queued_control_);
  climate.finish_("mode readback timeout", false);
  CHECK(climate.mode == climate::CLIMATE_MODE_HEAT && climate.system_mode.current_option() == "Heat");
  CHECK(!climate.pending.state && !climate.have_queued_control_ && !climate.verified.state);
  climate.system_mode.control(4);
  climate.set_transmit_enabled(false);
  climate.system_mode.control(4);  // Rejected commands cannot alter confirmed select state.
  CHECK(climate.stage_ == TestClimate::Stage::IDLE && !climate.pending.state);
  CHECK(climate.requested_.mode == -1 && climate.system_mode.current_option() == "Heat");
  climate.set_transmit_enabled(true);
  climate.system_mode.control(4);
  climate.fail_radio();
  CHECK(!climate.pending.state && climate.mode == climate::CLIMATE_MODE_HEAT);
  CHECK(climate.system_mode.current_option() == "Heat" && !climate.emergency.state);
}

static void expanded_target_range_and_deadband() {
  for (const auto pair : {std::array<int, 2>{55, 57}, {78, 80}, {55, 80}, {67, 73}}) {
    TestClimate climate;
    climate.publish_status_(actual());
    auto call = climate.make_call();
    call.set_target_temperature_low(pair[0]).set_target_temperature_high(pair[1]);
    climate.control(call);
    CHECK(climate.pending.state && climate.target_temperature_low == pair[0] &&
          climate.target_temperature_high == pair[1]);
    climate.read_status(actual());
    CHECK(climate.expected_.thermostat[7] == pair[0] && climate.expected_.thermostat[8] == pair[1]);
    climate.writes_acknowledged();
    climate.read_status(actual(pair[0], pair[1]));
    CHECK(climate.verified.state && !climate.pending.state);
  }
  TestClimate climate;
  climate.publish_status_(actual());
  for (const Control invalid :
       {targets(54), targets(81), targets(-1, 54), targets(-1, 81), targets(73), targets(-1, 71), targets(80, 55)}) {
    climate.request_control_(invalid);
    CHECK(!climate.pending.state && climate.target_temperature_low == 70 && climate.target_temperature_high == 74);
  }
  climate.request_control_(targets(-1, 80));
  climate.request_control_(targets(78));  // Validate against the unsent 80 F cool overlay, not actual 74 F.
  CHECK(climate.requested_.heat == 78 && climate.requested_.cool == 80);
  climate.request_control_(targets(79));
  CHECK(climate.requested_.heat == 78 && climate.target_temperature_low == 78);
  climate.read_status(actual());
  climate.request_control_(targets(76));      // Valid queued adjustment.
  climate.request_control_(targets(-1, 77));  // Invalid relative to queued heat, not actual 70 F.
  CHECK(climate.have_queued_control_ && climate.queued_control_.heat == 76 && climate.queued_control_.cool == -1);
  CHECK(climate.target_temperature_low == 76 && climate.target_temperature_high == 80);
  climate.request_control_(targets(-1, 78));
  CHECK(climate.queued_control_.cool == 78 && climate.target_temperature_high == 78);
  climate.finish_("cancel", false);

  // Unknown opposite target stays unknown; fresh status remains the final write gate.
  TestClimate unknown;
  unknown.request_control_(targets(78));
  CHECK(unknown.pending.state && unknown.target_temperature_low == 78 && std::isnan(unknown.target_temperature_high));
  unknown.read_status(actual());
  CHECK(!unknown.pending.state && !unknown.verified.state && unknown.write_count_ == 0);
  CHECK(unknown.target_temperature_low == 70 && unknown.target_temperature_high == 74);
}

static void confirmed_hold_deadlines() {
  TestClimate climate;
  text_sensor::TextSensor heat, cool;
  climate.set_heat_hold_until_sensor(&heat);
  climate.set_cool_hold_until_sensor(&cool);
  Status status = actual(70, 74, 1);
  status.thermostat[11] = 0x80;  // Midnight with the unrelated high bit set.
  status.thermostat[12] = 95;
  climate.publish_status_(status);
  CHECK(heat.state == "00:00" && cool.state == "23:45");
  status.thermostat[11] = 91;
  status.thermostat[12] = 92 | 0x80;
  status.thermostat[4] = 0x9a;
  climate.publish_status_(status);
  CHECK(heat.state == "00:00" && cool.state == "23:45");
  status.thermostat[4] = 0x5a;
  climate.publish_status_(status);
  CHECK(heat.state == "22:45" && cool.state == "23:00");
  status.thermostat[9] = (status.thermostat[9] & ~0x78) | (2 << 5);
  climate.publish_status_(status);
  CHECK(heat.state == "Following schedule" && cool.state == "Permanent hold");
}

static void passive_discovery_and_dynamic_slot_zero() {
  TestClimate climate;
  // Reversed map sends logical slot zero on rank49/channel121, not channel10.
  climate.clock_listen_.sync_us = micros() + 1000000;
  climate.stage_ = TestClimate::Stage::BEFORE;
  climate.schedule_();
  CHECK(climate.exchange_ == TestClimate::Exchange::SLOT);
  CHECK(climate.receive_candidate_ == 49);
  CHECK(candidate_channel(climate.receive_candidate_).channel == 121);
  CHECK(candidate_channel(climate.receive_candidate_).sync == 0x639c);
  climate.finish_("test finished", false);
  climate.rediscover_channels();
  CHECK(!climate.tx_allowed_() && !climate.hop_.ready());
  CHECK(climate.hop_.active_rank(0) == 49);  // Partial rebuild never replaces active map.
  climate.request_control_(targets(72));
  CHECK(climate.stage_ == TestClimate::Stage::IDLE && !climate.pending.state);
  climate.refresh();
  CHECK(climate.stage_ == TestClimate::Stage::IDLE);
  // A different permutation must also drive TX tuning, not just the reverse fixture.
  for (uint8_t slot = 0; slot < HOP_COUNT; ++slot) {
    const uint8_t rank = (slot * 17 + 7) % HOP_COUNT;
    CHECK(climate.hop_.observe(slot, rank, 10000 + slot));
    CHECK(climate.hop_.observe(slot, rank, 20000 + slot));
  }
  climate.set_transmit_enabled(true);
  climate.have_clock_ = true;
  climate.clock_received_ms_ = millis();
  climate.clock_listen_.sync_us = micros() + 1000000;
  climate.stage_ = TestClimate::Stage::BEFORE;
  climate.schedule_();
  CHECK(climate.receive_candidate_ == 7 && climate.exchange_ == TestClimate::Exchange::SLOT);
  CHECK(candidate_channel(climate.receive_candidate_).channel == 18);
  climate.finish_("test finished", false);
  climate.hop_.reset();
  climate.have_clock_ = true;
  climate.have_clock_counter_ = true;
  climate.clock_listen_ = {12, 0, micros() + 100000};
  climate.listen_for_clock_();
  CHECK(climate.receive_candidate_ == 0);
  climate.clock_listen_.sync_us += 15041992;
  climate.listen_for_clock_();
  CHECK(climate.receive_candidate_ == 1);  // Unknown-slot trial advances, not lockstep.
}

static Frame clock_frame(uint8_t slot, uint32_t c1, uint8_t copy = 0) {
  Frame frame;
  frame.size = 24;
  const uint8_t header[] = {0x15, 0x02, 0xff, 0xff, 0x38, 0x9e, 0x38, 0x9e, 0, 0, 0xff, 0x8a, 0x51};
  std::copy(std::begin(header), std::end(header), frame.data.begin());
  const uint32_t c2 = (c1 - 18 - copy * 242) & 0xffffff;
  const uint32_t counters[] = {c1, c2, (c2 - ((slot + 50 - copy) % 50) * 11) & 0xffffff};
  for (uint8_t i = 0; i < 3; ++i) {
    frame.data[13 + i * 3] = counters[i] >> 16;
    frame.data[14 + i * 3] = counters[i] >> 8;
    frame.data[15 + i * 3] = counters[i];
  }
  const uint16_t crc = crc16(frame.data.data(), frame.size - 2);
  frame.data[22] = crc >> 8;
  frame.data[23] = crc;
  return frame;
}

static void cold_scanner_learns_unknown_order() {
  now_us = 0;
  TestClimate cold(false);
  uint64_t burst = 0;
  uint8_t copy = 0;
  uint64_t due = 1000000;
  // One receiver sees only its tuned channel. Drive actual scan decisions,
  // rather than injecting each frame on the channel the learner needs.
  for (; now_us < 4ULL * 3600 * 1000000 && !cold.hop_.ready(); now_us += 1000) {
    cold.listen_for_clock_();
    if (now_us < due)
      continue;
    const uint8_t slot = (burst + copy) % HOP_COUNT;
    const uint8_t physical_rank = (slot * 17 + 7) % HOP_COUNT;
    if (cold.receive_candidate_ == physical_rank) {
      const uint32_t counter = (1000000 + burst * 61611 + copy * 242 + 18) & 0xffffff;
      cold.received_(clock_frame(slot, counter, copy), static_cast<uint32_t>(due));
    }
    if (++copy == 4) {
      copy = 0;
      ++burst;
    }
    due = 1000000 + (burst * 61611 + copy * 242) * 1000000ULL / 4096;
  }
  CHECK(cold.hop_.ready() && cold.hop_.confirmed() == HOP_COUNT);
  for (uint8_t slot = 0; slot < HOP_COUNT; ++slot)
    CHECK(cold.hop_.active_rank(slot) == (slot * 17 + 7) % HOP_COUNT);
}

static void live_learning_cache_and_duplicates() {
  ESPPreferences preferences;
  preference_records.clear();
  global_preferences = &preferences;
  TestClimate cold(false);
  cold.have_clock_ = false;
  CHECK(!cold.hop_.ready() && !cold.tx_allowed_());
  uint32_t counter = 1000;
  for (uint8_t pass = 0; pass < 2; ++pass) {
    for (uint8_t slot = 0; slot < HOP_COUNT; ++slot) {
      cold.receive_candidate_ = (slot * 17 + 7) % HOP_COUNT;
      const Frame frame = clock_frame(slot, counter);
      cold.received_(frame, micros());
      cold.received_(frame, micros());  // Duplicate FIFO/readback never adds evidence.
      counter += 61611;
      now_us += 15041992;  // 61611 ticks: preserve the measured counter/RF-time relationship.
    }
    CHECK(cold.hop_.confirmed() == (pass == 0 ? 0 : HOP_COUNT));
    CHECK(preference_records.size() == (pass == 0 ? 0 : 1));
  }
  CHECK(cold.hop_.ready() && cold.tx_allowed_());
  const auto saved = preference_records;
  TestClimate restored(false);
  CHECK(restored.hop_.cached() && !restored.hop_.ready());
  restored.receive_candidate_ = 7;
  restored.received_(clock_frame(0, counter), micros());
  restored.received_(clock_frame(0, counter), micros());
  CHECK(!restored.hop_.ready());
  restored.receive_candidate_ = 24;
  now_us += 59082;
  restored.received_(clock_frame(1, counter + 242), micros());
  CHECK(restored.hop_.ready());
  const uint32_t fresh_ms = restored.clock_received_ms_;
  now_us += 19000000;
  restored.received_(clock_frame(1, counter + 242), micros());
  CHECK(restored.clock_received_ms_ == fresh_ms);  // Replay cannot refresh TX clock.
  restored.stage_ = TestClimate::Stage::BEFORE;
  restored.schedule_();
  CHECK(restored.exchange_ == TestClimate::Exchange::CLOCK);
  restored.finish_("test complete", false);
  restored.rediscover_channels();
  restored.receive_candidate_ = 2;
  restored.received_(clock_frame(0, counter + 61611), micros());
  CHECK(preference_records == saved && !restored.hop_.ready());
  restored.receive_candidate_ = 3;
  now_us += 15041992;
  restored.received_(clock_frame(0, counter + 123222), micros());
  CHECK(restored.hop_.conflicted() && !restored.tx_allowed_());
  CHECK(preference_records == saved);  // Conflict cannot overwrite the last good map.
  preference_records.begin()->second[12] ^= 1;
  TestClimate corrupt(false);
  CHECK(!corrupt.hop_.cached() && !corrupt.hop_.ready());
  global_preferences = nullptr;
  preference_records.clear();
}

static void clock_epoch_reacquisition() {
  now_us = 1000000;
  TestClimate climate;
  climate.have_clock_ = climate.have_clock_counter_ = false;
  uint32_t counter = 4000000;
  climate.receive_candidate_ = 49;
  climate.received_(clock_frame(0, counter), micros());
  CHECK(!climate.fresh_clock_());
  now_us += 59082;
  counter += 242;
  climate.receive_candidate_ = 48;
  climate.received_(clock_frame(1, counter), micros());
  CHECK(climate.fresh_clock_() && climate.hop_.ready());

  // More than the half-counter period: a distinct timed pair must recover
  // without waiting for a 4096-second counter wrap or discarding the full map.
  now_us += 2100000000U;
  counter = (counter + 2100U * 4096) & 0xffffff;
  climate.receive_candidate_ = 47;
  climate.received_(clock_frame(2, counter), micros());
  CHECK(!climate.fresh_clock_() && climate.hop_.active_rank(0) == 49);
  climate.received_(clock_frame(2, counter), micros());
  CHECK(!climate.fresh_clock_());
  now_us += 59082;
  counter = (counter + 242) & 0xffffff;
  climate.receive_candidate_ = 46;
  climate.received_(clock_frame(3, counter), micros());
  CHECK(climate.fresh_clock_() && climate.hop_.ready() && !climate.hop_.conflicted());

  // EIM reset: lower counters alone are not fresh; reject duplicate and
  // incorrectly timed copies before accepting a new, consistent pair.
  now_us += 15041992;
  climate.receive_candidate_ = 49;
  climate.received_(clock_frame(0, 1000), micros());
  CHECK(!climate.fresh_clock_());
  climate.received_(clock_frame(0, 1000), micros());
  CHECK(!climate.fresh_clock_());
  now_us += 500000;
  climate.receive_candidate_ = 48;
  climate.received_(clock_frame(1, 1242), micros());
  CHECK(!climate.fresh_clock_());
  now_us += 59082;
  climate.receive_candidate_ = 47;
  climate.received_(clock_frame(2, 1484), micros());
  CHECK(climate.fresh_clock_() && climate.hop_.ready() && !climate.hop_.conflicted());

  TestClimate conflict(false);
  conflict.receive_candidate_ = 1;
  conflict.received_(clock_frame(0, 1000), micros());
  now_us += 59082;
  conflict.receive_candidate_ = 2;
  conflict.received_(clock_frame(1, 1000), micros());
  CHECK(conflict.hop_.conflicted() && !conflict.fresh_clock_());
  CHECK(conflict.hop_.known() == 1 && conflict.hop_.confirmed() == 0);
}

int main() {
  cold_scanner_learns_unknown_order();
  clock_epoch_reacquisition();
  live_learning_cache_and_duplicates();
  passive_discovery_and_dynamic_slot_zero();
  confirmed_hold_deadlines();
  measured_climate_traits_and_initial_unknown();
  measurements_are_independently_guarded();
  measurements_during_optimism_verification_and_rollback();
  system_mode_selection_and_no_op();
  system_mode_merging_queue_and_failure();
  expanded_target_range_and_deadband();
  emergency_heat_readback_and_optimism();
  immediate_optimism_and_rollback();
  merged_and_queued_controls();
  mismatched_readback_rolls_back_to_latest_actual();
  invalid_calls_are_atomic();
  schedule_merges_and_no_op();
  refresh_and_pending_status();
  transmit_disable_and_radio_failure();
  stale_prewrite_reply(1000000);
  stale_prewrite_reply(200);  // Timestamp comparison remains valid across micros() wrap.
  CHECK(transport_sends == 0);
  std::puts("RedLINK climate control tests passed");
}
