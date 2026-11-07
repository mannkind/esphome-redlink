#include "redlink.h"
#include "esphome/core/log.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace esphome::redlink {
static const char *const TAG = "redlink";
static constexpr double TICK_US = 1000000.0 / 4096.0;
static constexpr double CYCLE_US = 550 * TICK_US;
static constexpr double PREAMBLE_US = 128000000.0 / 38383.0;
static constexpr uint32_t ACTUAL_STATUS_TIMEOUT_MS = 180000;
static constexpr uint32_t PAIRING_MAGIC = 0x524c0001;
// Option order matches climate.py; select state is always confirmed actual status.
static constexpr std::array<uint8_t, 5> SYSTEM_MODE_CODES{4, 1, 3, 2, 0};

namespace {
bool has_control(const Control &c) {
  return c.heat != -1 || c.cool != -1 || c.mode != -1 || c.fan != -1 || c.hold != -1 || c.until != -1;
}

bool valid_control(const Control &c) {
  return (c.heat == -1 || (c.heat >= 55 && c.heat <= 80)) && (c.cool == -1 || (c.cool >= 55 && c.cool <= 80)) &&
         (c.mode == -1 || (c.mode >= 0 && c.mode <= 4)) && (c.fan == -1 || c.fan == 0 || c.fan == 1 || c.fan == 3) &&
         c.hold >= -1 && c.hold <= 2 && c.until >= -1 && c.until <= 95 && (c.until == -1 || c.hold == 1) &&
         (c.hold != 0 || (c.heat == -1 && c.cool == -1));
}

Control merge_control(Control older, const Control &newer) {
  // Resume schedule supersedes unsent targets. A later explicit target supersedes
  // an unsent schedule selection, just as it does on the thermostat itself.
  if (newer.hold == 0)
    older.heat = older.cool = -1;
  else if (older.hold == 0 && (newer.heat != -1 || newer.cool != -1)) {
    older.hold = -1;
    older.until = -1;
  }
  older.merge(newer);
  return older;
}
}  // namespace

void RedLinkSystemModeSelect::control(size_t index) {
  if (index >= SYSTEM_MODE_CODES.size())
    return;
  parent_->set_system_mode(SYSTEM_MODE_CODES[index]);
}

void RedLinkSystemModeSelect::publish_mode(uint8_t mode) {
  for (size_t i = 0; i < SYSTEM_MODE_CODES.size(); ++i) {
    if (SYSTEM_MODE_CODES[i] == mode) {
      publish_state(i);
      return;
    }
  }
}

void RedLinkClimate::setup() {
  RedLinkRadio::setup();
  this->set_supported_custom_fan_modes({"circulate"});
  this->set_supported_custom_presets({"schedule", "temporary", "permanent", "mixed"});
  // Never restore a climate call: boot must not rewrite the thermostat.
  // This marker records a registry read, not climate state or proof of current radio reachability.
  if (global_preferences != nullptr) {
    pairing_preference_ =
        global_preferences->make_preference<PairingMarker>(PAIRING_MAGIC ^ (uint32_t(network_) << 16) ^ gateway_);
    PairingMarker marker{};
    paired_state_ = pairing_preference_.load(&marker) && marker.magic == PAIRING_MAGIC && marker.network == network_ &&
                    marker.gateway == gateway_;
    pairing_saved_ = paired_state_;
  }
  if (paired_ != nullptr)
    paired_->publish_state(paired_state_);
  report_pairing_(paired_state_ ? "Previously verified; not checked this boot"
                                : "Not verified; Refresh Status checks the existing association");
  high_frequency_.start();
  report_("Transmit disabled; passive clock acquisition");
  if (acknowledged_ != nullptr)
    acknowledged_->publish_state(false);
  if (verified_ != nullptr)
    verified_->publish_state(false);
  if (clock_locked_ != nullptr)
    clock_locked_->publish_state(false);
  if (command_pending_ != nullptr)
    command_pending_->publish_state(false);
}

void RedLinkClimate::dump_config() {
  RedLinkRadio::dump_config();
  LOG_CLIMATE("", "RedLINK", this);
  LOG_SELECT("", "System Mode", system_mode_);
  ESP_LOGCONFIG(TAG, "  Network: %04x, configured gateway identity: %04x", network_, gateway_);
  ESP_LOGCONFIG(TAG, "  TX advance: %ld us; slot 0 only; experimental RF timing", (long) tx_advance_us_);
  ESP_LOGCONFIG(TAG, "  RX: search spaced channels, then follow all 50 clock slots; 18 s clock freshness");
}

climate::ClimateTraits RedLinkClimate::traits() {
  climate::ClimateTraits t;
  t.add_feature_flags(climate::CLIMATE_SUPPORTS_TWO_POINT_TARGET_TEMPERATURE |
                      climate::CLIMATE_REQUIRES_TWO_POINT_TARGET_TEMPERATURE |
                      climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE | climate::CLIMATE_SUPPORTS_CURRENT_HUMIDITY);
  t.set_supported_modes({climate::CLIMATE_MODE_OFF, climate::CLIMATE_MODE_HEAT, climate::CLIMATE_MODE_COOL,
                         climate::CLIMATE_MODE_HEAT_COOL});
  t.set_supported_fan_modes({climate::CLIMATE_FAN_AUTO, climate::CLIMATE_FAN_ON});
  t.set_temperature_unit(TemperatureUnit::FAHRENHEIT);
  t.set_visual_min_temperature(55.0f);
  t.set_visual_max_temperature(80.0f);
  t.set_visual_temperature_step(1.0f);
  return t;
}

void RedLinkClimate::report_(const char *message) {
  ESP_LOGI(TAG, "%s", message);
  if (result_ != nullptr)
    result_->publish_state(message);
}

void RedLinkClimate::report_pairing_(const char *message) {
  ESP_LOGI(TAG, "Pairing: %s", message);
  if (pairing_status_ != nullptr)
    pairing_status_->publish_state(message);
}

bool RedLinkClimate::save_pairing_(bool verified) {
  const PairingMarker marker{verified ? PAIRING_MAGIC : 0, network_, gateway_};
  return global_preferences != nullptr && pairing_preference_.save(&marker) && global_preferences->sync();
}

bool RedLinkClimate::tx_allowed_() const { return !this->is_failed() && transmit_enabled_ && !receive_only_; }

void RedLinkClimate::report_pairing_stage_() {
  switch (enrollment_.state()) {
    case GatewayEnrollment::State::DISCOVER:
      report_pairing_("Discovering; thermostat Add Device must be open");
      break;
    case GatewayEnrollment::State::ANNOUNCE:
      report_pairing_("Announcing gateway");
      break;
    case GatewayEnrollment::State::REGISTER:
      report_pairing_("Registering gateway; replacement may occur");
      break;
    case GatewayEnrollment::State::COMMIT:
      report_pairing_("Committing gateway association");
      break;
    case GatewayEnrollment::State::VERIFY:
      report_pairing_("Reading gateway registry; ACK is not proof");
      break;
    default:
      break;
  }
}

void RedLinkClimate::pair_gateway() {
  if (!tx_allowed_()) {
    report_("Pairing rejected: radio unavailable, receive-only, or transmit disabled");
    return;
  }
  if (stage_ != Stage::IDLE || exchange_ == Exchange::TX) {
    report_("Pairing rejected: transaction busy");
    return;
  }
  stage_ = Stage::PAIRING;
  transaction_started_ms_ = millis();
  enrollment_.start(network_, gateway_);
  if (acknowledged_ != nullptr)
    acknowledged_->publish_state(false);
  if (verified_ != nullptr)
    verified_->publish_state(false);
  report_("Explicit gateway pairing started; HVAC mode is not changed");
  report_pairing_stage_();
  begin_exchange_(enrollment_.outgoing());
}

bool RedLinkClimate::fresh_clock_() const {
  // Consecutive broadcast starts are ~15.042 seconds apart, not less than 15.
  return have_clock_ && millis() - clock_received_ms_ < 18000;
}

void RedLinkClimate::listen_for_clock_() {
  // Never cut off an in-progress frame to move to another receive channel.
  if (receiving_())
    return;
  const uint32_t now = millis();
  if (have_clock_ && now - clock_received_ms_ < 20000) {
    // Advance past missed copies instead of waiting on a channel whose packet
    // has already passed. Eight ms covers drift without eating the 59-ms gap.
    const uint32_t now_us = micros();
    while (static_cast<int32_t>(now_us - clock_listen_.sync_us) > 8000) {
      clock_listen_.advance();
      ESP_LOGI(TAG, "Clock copy missed; following slot %u, copy %u", clock_listen_.slot, clock_listen_.copy);
    }
    listen_slot_(clock_listen_.slot);
    scan_changed_ms_ = now;
    return;
  }
  if (now - scan_changed_ms_ < 15400)
    return;
  // Cover a complete 61611-tick broadcast period plus its four copies.
  // Step five slots so search does not follow the EIM's own one-slot progression.
  listen_slot_((receive_slot_ + 5) % HOP_CHANNELS.size());
  scan_changed_ms_ = now;
}

void RedLinkClimate::set_transmit_enabled(bool enabled) {
  transmit_enabled_ = enabled;
  if (enabled) {
    refresh();
    return;
  }
  // A packet already on air cannot be recalled. Do not interrupt TX mid-frame.
  if (stage_ != Stage::IDLE || have_queued_control_) {
    finish_("Disabled; any transmitted write may have applied", false);
    return;
  }
  report_("Transmit disabled; passive clock acquisition");
}

void RedLinkClimate::on_radio_failure_() {
  // Component failure can stop loop() entirely, so rollback at the fault source.
  have_clock_ = clock_published_ = false;
  if (clock_locked_ != nullptr)
    clock_locked_->publish_state(false);
  finish_("Radio failed; transmitted packets may have applied", false);
}

bool RedLinkClimate::start_(const Control &control) {
  if (!tx_allowed_()) {
    report_("Request rejected: radio unavailable, receive-only, or transmit disabled");
    return false;
  }
  if (!valid_control(control)) {
    report_("Request rejected: invalid control combination");
    return false;
  }
  if (stage_ != Stage::IDLE || exchange_ == Exchange::TX) {
    ESP_LOGI(TAG, "Trace t=%lu busy stage=%u exchange=%u elapsed=%lu ms", (unsigned long) millis(), unsigned(stage_),
             unsigned(exchange_), (unsigned long) (millis() - transaction_started_ms_));
    return false;
  }
  requested_ = control;
  complete_association_ = false;
  stage_ = Stage::BEFORE;
  transaction_started_ms_ = millis();
  settle_started_ms_ = 0;
  write_count_ = write_index_ = 0;
  all_write_acks_ = true;
  if (acknowledged_ != nullptr)
    acknowledged_->publish_state(false);
  if (verified_ != nullptr)
    verified_->publish_state(false);
  publish_climate_();
  report_("Reading fresh EIM status");
  begin_exchange_(status_query(network_, gateway_));
  return true;
}

bool RedLinkClimate::valid_requested_targets_(const Control &control) const {
  if (control.heat == -1 && control.cool == -1 && control.hold != 0)
    return true;
  int heat = confirmed_status_.complete() ? confirmed_status_.thermostat[7] : -1;
  int cool = confirmed_status_.complete() ? confirmed_status_.thermostat[8] : -1;
  const auto apply = [&](const Control &candidate) {
    if (candidate.hold == 0) {
      heat = confirmed_status_.complete() ? confirmed_status_.thermostat[17] : -1;
      cool = confirmed_status_.complete() ? confirmed_status_.thermostat[18] : -1;
      return;
    }
    if (candidate.heat != -1)
      heat = candidate.heat;
    if (candidate.cool != -1)
      cool = candidate.cool;
  };
  // A write already in flight is the baseline for its queue; an unsent request
  // is merged instead, so schedule/target supersession matches admission below.
  if (stage_ == Stage::WRITE || stage_ == Stage::VERIFY)
    apply(requested_);
  const Control pending = stage_ == Stage::BEFORE ? requested_ : (have_queued_control_ ? queued_control_ : Control{});
  apply(merge_control(pending, control));
  return (heat == -1 || (heat >= 55 && heat <= 80)) && (cool == -1 || (cool >= 55 && cool <= 80)) &&
         (heat == -1 || cool == -1 || cool - heat >= 2);
}

void RedLinkClimate::request_control_(const Control &control) {
  if (!has_control(control) || !valid_control(control)) {
    report_("Control rejected: empty or invalid control combination");
    return;
  }
  if (!tx_allowed_()) {
    report_("Request rejected: radio unavailable, receive-only, or transmit disabled");
    return;
  }
  if (!valid_requested_targets_(control)) {
    report_("Control rejected: effective targets require 55..80 F and a 2 F deadband");
    return;
  }
  if (stage_ == Stage::IDLE) {
    if (exchange_ == Exchange::TX) {
      report_("Control rejected: previous packet still on air");
      return;
    }
    const Control next = have_queued_control_ ? merge_control(queued_control_, control) : control;
    have_queued_control_ = false;
    start_(next);
    return;
  }
  if (stage_ == Stage::BEFORE && !complete_association_) {
    // No write has left yet: merge only the accepted explicit fields.
    requested_ = merge_control(requested_, control);
    report_("Updated requested controls; reading fresh EIM status");
  } else if (stage_ == Stage::WRITE || stage_ == Stage::VERIFY) {
    queued_control_ = merge_control(have_queued_control_ ? queued_control_ : Control{}, control);
    have_queued_control_ = true;
    report_("Latest controls queued; waiting for current write verification");
  } else {
    report_("Control rejected: gateway association operation in progress");
    return;
  }
  if (acknowledged_ != nullptr)
    acknowledged_->publish_state(false);
  if (verified_ != nullptr)
    verified_->publish_state(false);
  publish_climate_();
}

void RedLinkClimate::refresh() {
  if (!start_(Control{}) || paired_state_)
    return;
  stage_ = Stage::CHECK_REGISTRY;
  registry_confirmed_ = false;
  report_pairing_("Checking existing gateway registry; no enrollment writes");
  begin_exchange_(gateway_registry_query(network_, gateway_));
}

void RedLinkClimate::complete_gateway_association() {
  // Admission must be independently demonstrated before completing its registry entry.
  if (!start_(Control{}))
    return;
  complete_association_ = true;
  report_pairing_("Checking actual status before completing association; no registration");
}

void RedLinkClimate::temporary_hold_until(int hour, int minute) {
  if (hour < 0 || hour > 23 || minute < 0 || minute > 45 || minute % 15 != 0) {
    ESP_LOGW(TAG, "Hold deadline requires hour 0..23 and minute 0/15/30/45");
    return;
  }
  Control c;
  c.hold = 1;
  c.until = hour * 4 + minute / 15;
  request_control_(c);
}

void RedLinkClimate::set_system_mode(int mode) {
  Control c;
  c.mode = mode;
  request_control_(c);
}

void RedLinkClimate::control(const climate::ClimateCall &call) {
  ESP_LOGI(TAG, "Trace t=%lu climate heat=%.1f cool=%.1f mode=%d fan=%d custom_fan=%s preset=%s (nan/-1=unset)",
           (unsigned long) millis(),
           call.get_target_temperature_low().has_value() ? *call.get_target_temperature_low() : NAN,
           call.get_target_temperature_high().has_value() ? *call.get_target_temperature_high() : NAN,
           call.get_mode().has_value() ? int(*call.get_mode()) : -1,
           call.get_fan_mode().has_value() ? int(*call.get_fan_mode()) : -1,
           call.has_custom_fan_mode() ? call.get_custom_fan_mode().c_str() : "-",
           call.has_custom_preset() ? call.get_custom_preset().c_str() : "-");
  Control c;
  auto temperature = [](float value, int minimum, int maximum, int &out) {
    const float f = value;  // Native API and RedLINK both use Fahrenheit.
    if (!std::isfinite(f))
      return false;
    const float rounded = std::round(f);
    if (std::fabs(f - rounded) > 0.05f || rounded < minimum || rounded > maximum)
      return false;
    out = static_cast<int>(rounded);
    return true;
  };
  if (call.get_target_temperature_low().has_value() &&
      !temperature(*call.get_target_temperature_low(), 55, 80, c.heat)) {
    ESP_LOGW(TAG, "Rejected heat target: integer 55..80 F required");
    return;
  }
  if (call.get_target_temperature_high().has_value() &&
      !temperature(*call.get_target_temperature_high(), 55, 80, c.cool)) {
    ESP_LOGW(TAG, "Rejected cool target: integer 55..80 F required");
    return;
  }
  if (call.get_mode().has_value()) {
    switch (*call.get_mode()) {
      case climate::CLIMATE_MODE_HEAT:
        c.mode = 1;
        break;
      case climate::CLIMATE_MODE_OFF:
        c.mode = 2;
        break;
      case climate::CLIMATE_MODE_COOL:
        c.mode = 3;
        break;
      case climate::CLIMATE_MODE_HEAT_COOL:
        c.mode = 4;
        break;
      default:
        return;
    }
  }
  if (call.get_fan_mode().has_value()) {
    switch (*call.get_fan_mode()) {
      case climate::CLIMATE_FAN_AUTO:
        c.fan = 0;
        break;
      case climate::CLIMATE_FAN_ON:
        c.fan = 1;
        break;
      default:
        return;
    }
  }
  if (call.has_custom_fan_mode()) {
    if (call.get_custom_fan_mode() != "circulate")
      return;
    c.fan = 3;
  }
  if (call.has_custom_preset()) {
    auto preset = call.get_custom_preset();
    if (preset == "schedule")
      c.hold = 0;
    else if (preset == "temporary")
      c.hold = 1;
    else if (preset == "permanent")
      c.hold = 2;
    else
      return;  // mixed is read-only, never map it to a write.
  }
  request_control_(c);
}

void RedLinkClimate::begin_exchange_(const Frame &frame) {
  if (stage_ == Stage::WRITE && write_index_ == 0 && !fresh_clock_()) {
    finish_("Clock expired after status; no write sent", false);
    return;
  }
  outgoing_ = frame;
  attempts_ = 0;
  saw_ack_ = false;
  received_status_ = Status{};
  clock_wait_started_ms_ = millis();
  exchange_ = Exchange::CLOCK;
}

void RedLinkClimate::schedule_() {
  if (!tx_allowed_()) {
    finish_("TX authorization removed; no further packets sent", false);
    return;
  }
  if (!fresh_clock_()) {
    if (stage_ == Stage::WRITE)
      finish_("Clock expired; no further writes sent", false);
    else {
      if (exchange_ != Exchange::CLOCK)
        clock_wait_started_ms_ = millis();
      exchange_ = Exchange::CLOCK;
    }
    return;
  }
  // A status/write exchange occupies slot zero through its reply window. Do not
  // start one across a predicted clock copy: missing all four copies would make
  // the anchor expire and delay the next command while we reacquire it.
  const uint32_t before_tune = micros();
  const uint32_t since_previous_tx = before_tune - last_tx_us_;
  const uint32_t spacing_us = sent_any_ && since_previous_tx < 1000000 ? 1000000 - since_previous_tx : 0;
  const uint32_t reply_budget_us = stage_ == Stage::WRITE ? 200000 : 450000;  // TX timeout + reply window.
  const int32_t until_clock = static_cast<int32_t>(clock_listen_.sync_us - before_tune);
  const uint32_t occupied_us = spacing_us + static_cast<uint32_t>(std::ceil(CYCLE_US)) + reply_budget_us + 8000;
  if (receiving_() || (until_clock >= -8000 && until_clock <= static_cast<int32_t>(occupied_us))) {
    // Keep the radio following clock channels until the burst has passed.
    // Only a real clock packet refreshes the anchor; never extend its lifetime.
    exchange_ = Exchange::CLOCK;
    return;
  }
  // Retune/calibrate before calculating the deadline, never during the timed TX.
  if (!listen_slot_(0)) {
    finish_("Could not tune the verified transmit channel; no packet sent", false);
    return;
  }
  const uint32_t now = micros();
  // Clock counters refer to the FIRST sync bit, not FIFO delivery or preamble.
  const double base = 18 * TICK_US - clock_phase_ticks_ * TICK_US - PREAMBLE_US - tx_advance_us_;
  const int32_t elapsed = static_cast<int32_t>(now - clock_sync_us_);
  const uint32_t since_tx = now - last_tx_us_;
  const int32_t spacing = sent_any_ && since_tx < 1000000 ? 1000000 - since_tx : 0;
  const double earliest = elapsed + std::max<int32_t>(spacing, 2000);
  const double cycle = std::ceil((earliest - base) / CYCLE_US);
  due_us_ = clock_sync_us_ + static_cast<int32_t>(std::llround(base + cycle * CYCLE_US));
  exchange_ = Exchange::SLOT;
}

void RedLinkClimate::received_(const Frame &frame, uint32_t sync_us) {
  ESP_LOGI(TAG, "RX CRC-valid: %u bytes, type 0x%02x, channel %u", unsigned(frame.size), frame.data[1],
           HOP_CHANNELS[receive_slot_].channel);
  uint8_t slot;
  uint32_t phase;
  ClockListenWindow next;
  if (clock_slot(frame, network_, slot, phase, next) && slot == receive_slot_) {
    clock_sync_us_ = sync_us;
    clock_phase_ticks_ = phase;
    clock_received_ms_ = millis();
    have_clock_ = true;
    clock_listen_ = next;
    clock_listen_.sync_us += sync_us;
    ESP_LOGI(TAG, "EIM clock acquired: slot %u, phase %u ticks; following slot %u", slot, unsigned(phase), next.slot);
  }
  // Replies belong only to this receive window, never to another query/attempt.
  if (stage_ == Stage::IDLE || exchange_ != Exchange::RX)
    return;
  // A frame buffered before this attempt cannot verify a later write. Compare
  // sync time (not FIFO delivery time); signed subtraction also handles wrap.
  if (static_cast<int32_t>(sync_us - last_tx_us_) < 0)
    return;
  if (peer_reply(frame, network_, gateway_) && frame.size == 21 &&
      (stage_ == Stage::CHECK_REGISTRY ||
       (stage_ == Stage::PAIRING && enrollment_.state() == GatewayEnrollment::State::VERIFY))) {
    ESP_LOGI(TAG, "Registry reply type=%02X block=%02X%02X%02X%02X%02X%02X%02X%02X; expected gateway=%04X",
             frame.data[1], frame.data[11], frame.data[12], frame.data[13], frame.data[14], frame.data[15],
             frame.data[16], frame.data[17], frame.data[18], gateway_);
  }
  if (stage_ == Stage::CHECK_REGISTRY) {
    registry_confirmed_ |= registry_reply(frame, network_, gateway_);
    return;
  }
  if (stage_ == Stage::PAIRING) {
    enrollment_.receive(frame);
    return;
  }
  if (frame.data[1] == 0x22) {
    if (acknowledgement(frame, network_, gateway_)) {
      saw_ack_ = true;
      ESP_LOGI(TAG, "Trace t=%lu ACK stage=%u attempt=%u elapsed=%lu ms", (unsigned long) millis(), unsigned(stage_),
               unsigned(attempts_), (unsigned long) (millis() - transaction_started_ms_));
    }
    return;
  }
  if (stage_ == Stage::WRITE || !peer_reply(frame, network_, gateway_))
    return;
  if (!collect_status(frame, received_status_))
    return;  // collect_status validated every block/separator; pending blocks invalidate,
  // rather than replace, the corresponding accumulated actual status.
  const size_t end = frame.size - 2;
  for (size_t position = 11; position < end;) {
    const uint8_t *block = frame.data.data() + position;
    const size_t size = (block[0] & 0x7f) + 1;
    if (size == 31 && (block[1] & 0x3f) == 1 && block[2] == 0xff && block[3] == 0 &&
        (block[4] == 0x5a || block[4] == 0x9a)) {
      ESP_LOGI(TAG, "Trace t=%lu status thermostat=%s heat=%u cool=%u stage=%u elapsed=%lu ms",
               (unsigned long) millis(), block[4] == 0x5a ? "actual" : "pending", unsigned(block[7]),
               unsigned(block[8]), unsigned(stage_), (unsigned long) (millis() - transaction_started_ms_));
    } else if (size == 10 && (block[1] & 0x3f) == 0x22 && block[2] == 0xc0 && block[3] == 0 &&
               (block[4] == 0x45 || block[4] == 0x85)) {
      ESP_LOGI(TAG, "Trace t=%lu status fan=%s value=%u stage=%u elapsed=%lu ms", (unsigned long) millis(),
               block[4] == 0x45 ? "actual" : "pending", unsigned(block[9] & 3), unsigned(stage_),
               (unsigned long) (millis() - transaction_started_ms_));
    }
    position += size;
    if (position < end)
      ++position;
  }
}

void RedLinkClimate::finish_(const char *result, bool verified) {
  ESP_LOGI(TAG, "Trace t=%lu complete stage=%u verified=%u elapsed=%lu ms: %s", (unsigned long) millis(),
           unsigned(stage_), unsigned(verified), (unsigned long) (millis() - transaction_started_ms_), result);
  if (stage_ == Stage::PAIRING) {
    const bool uncertain = enrollment_.may_have_changed();
    enrollment_.fail();
    report_pairing_(uncertain
                        ? "Pairing unverified; gateway may have been replaced; no rollback attempted"
                        : (paired_state_ ? "Pairing stopped before registration; previous saved association unchanged"
                                         : "Pairing stopped before registration; no association verified"));
  } else if (stage_ == Stage::PAIR_STATUS && !verified) {
    report_pairing_(pairing_saved_ ? "Registry verified and saved; fresh HVAC status unavailable"
                                   : "Registry verified, not saved; fresh HVAC status unavailable");
  } else if (stage_ == Stage::CHECK_REGISTRY) {
    report_pairing_("Registry recheck unverified; no association write attempted");
  }
  // An uncertain write must not trigger another queued write automatically.
  if (!verified)
    have_queued_control_ = false;
  requested_ = Control{};
  stage_ = Stage::IDLE;
  last_refresh_ms_ = millis();
  if (verified_ != nullptr)
    verified_->publish_state(verified && !have_queued_control_);
  publish_climate_();
  report_(result);
}

void RedLinkClimate::publish_status_(const Status &s) {
  if (!actual_status(s))
    return;
  confirmed_status_ = s;
  // Observed whole-degree Fahrenheit room readings are duplicated in actual status.
  // Reject untested encodings per measurement, without discarding control context.
  current_temperature = s.thermostat[6] == s.thermostat[24] && s.thermostat[6] <= 127 ? s.thermostat[6] : NAN;
  current_humidity = s.thermostat[22] <= 100 ? s.thermostat[22] : NAN;
  for (size_t i = 0; i < 2; ++i) {
    auto *sensor = i == 0 ? heat_hold_until_ : cool_hold_until_;
    if (sensor == nullptr)
      continue;
    const uint8_t hold = (s.thermostat[9] >> (3 + 2 * i)) & 3;
    const uint8_t quarter = s.thermostat[11 + i] & 0x7f;
    char deadline[6];
    const char *value = "Unknown";
    if (hold == 0)
      value = "Following schedule";
    else if (hold == 2)
      value = "Permanent hold";
    else if (hold == 1 && quarter < 96) {
      std::snprintf(deadline, sizeof(deadline), "%02u:%02u", static_cast<unsigned>(quarter / 4),
                    static_cast<unsigned>((quarter % 4) * 15));
      value = deadline;
    }
    if (sensor->state != value)
      sensor->publish_state(value);
  }
  // These entities report confirmed mode only, never a request, ACK, or pending-write echo.
  if (emergency_heat_ != nullptr)
    emergency_heat_->publish_state((s.thermostat[19] & 7) == 0);
  if (system_mode_ != nullptr)
    system_mode_->publish_mode(s.thermostat[19] & 7);
  publish_climate_();
}

bool RedLinkClimate::command_pending_awaiting_() const {
  return have_queued_control_ ||
         ((stage_ == Stage::BEFORE || stage_ == Stage::WRITE || stage_ == Stage::VERIFY) && has_control(requested_));
}

void RedLinkClimate::apply_requested_(const Control &c) {
  if (c.heat != -1)
    target_temperature_low = c.heat;
  if (c.cool != -1)
    target_temperature_high = c.cool;
  switch (c.mode) {
    case 0:  // ESPHome has no dedicated Emergency Heat HVAC mode.
    case 1:
      mode = climate::CLIMATE_MODE_HEAT;
      break;
    case 2:
      mode = climate::CLIMATE_MODE_OFF;
      break;
    case 3:
      mode = climate::CLIMATE_MODE_COOL;
      break;
    case 4:
      mode = climate::CLIMATE_MODE_HEAT_COOL;
      break;
    default:
      break;
  }
  if (c.fan == 3)
    this->set_custom_fan_mode_("circulate");
  else if (c.fan != -1)
    this->set_fan_mode_(c.fan ? climate::CLIMATE_FAN_ON : climate::CLIMATE_FAN_AUTO);
  if (c.hold == 0) {
    target_temperature_low = confirmed_status_.complete() ? confirmed_status_.thermostat[17] : NAN;
    target_temperature_high = confirmed_status_.complete() ? confirmed_status_.thermostat[18] : NAN;
    this->set_custom_preset_("schedule");
  } else if (c.hold == 1)
    this->set_custom_preset_("temporary");
  else if (c.hold == 2)
    this->set_custom_preset_("permanent");
  else if ((c.heat != -1 || c.cool != -1) && this->get_custom_preset() == "schedule")
    this->set_custom_preset_("temporary");
}

void RedLinkClimate::publish_climate_() {
  // Rebuild from confirmed readback every time; never let an optimistic value
  // become the rollback baseline or feed write preparation.
  const Status &s = confirmed_status_;
  target_temperature_low = s.complete() ? s.thermostat[7] : NAN;
  target_temperature_high = s.complete() ? s.thermostat[8] : NAN;
  mode = climate::CLIMATE_MODE_OFF;
  fan_mode.reset();
  preset.reset();
  this->clear_custom_fan_mode_();
  this->clear_custom_preset_();
  if (s.complete()) {
    Control confirmed;
    confirmed.mode = s.thermostat[19] & 7;
    confirmed.fan = s.fan[9] & 3;
    apply_requested_(confirmed);
    const unsigned heat_hold = (s.thermostat[9] >> 3) & 3;
    const unsigned cool_hold = (s.thermostat[9] >> 5) & 3;
    const char *hold = "mixed";
    if (heat_hold == cool_hold) {
      if (heat_hold == 0)
        hold = "schedule";
      else if (heat_hold == 1)
        hold = "temporary";
      else if (heat_hold == 2)
        hold = "permanent";
    }
    this->set_custom_preset_(hold);
  }
  if (stage_ == Stage::BEFORE || stage_ == Stage::WRITE || stage_ == Stage::VERIFY)
    apply_requested_(requested_);
  if (have_queued_control_)
    apply_requested_(queued_control_);
  if (command_pending_ != nullptr)
    command_pending_->publish_state(command_pending_awaiting_());
  publish_state();
}

void RedLinkClimate::registry_verified_() {
  // Persist only the exact directed registry read, never an ACK or a status reply.
  pairing_saved_ = save_pairing_(true);
  paired_state_ = true;
  if (paired_ != nullptr)
    paired_->publish_state(true);
  stage_ = Stage::PAIR_STATUS;
  transaction_started_ms_ = millis();
  settle_started_ms_ = millis();
  report_pairing_(pairing_saved_ ? "Registry verified and saved; reading actual HVAC status"
                                 : "Registry verified but save failed; reading actual HVAC status");
  report_("Gateway registry verified; checking fresh climate state");
  begin_exchange_(status_query(network_, gateway_));
}

void RedLinkClimate::finish_window_() {
  if (stage_ == Stage::CHECK_REGISTRY) {
    if (registry_confirmed_)
      registry_verified_();
    else if (attempts_ < 5)
      schedule_();
    else {
      report_pairing_("Registry did not confirm this gateway; reading HVAC status without enrollment writes");
      stage_ = Stage::BEFORE;
      settle_started_ms_ = 0;
      begin_exchange_(status_query(network_, gateway_));
    }
    return;
  }
  if (stage_ == Stage::PAIRING) {
    const auto previous = enrollment_.state();
    enrollment_.finish_window();
    if (enrollment_.state() == GatewayEnrollment::State::FAILED) {
      finish_("Pairing reply timeout; no further enrollment packets sent", false);
    } else if (enrollment_.verified()) {
      registry_verified_();
    } else if (enrollment_.state() != previous) {
      report_pairing_stage_();
      if (enrollment_.state() == GatewayEnrollment::State::REGISTER && paired_state_) {
        // Clear only our historical marker, before scheduling a possibly replacing registration.
        // Flash writes must never delay a packet after its slot has been selected.
        if (!save_pairing_(false)) {
          finish_("Could not invalidate saved pairing; registration not sent", false);
          return;
        }
        paired_state_ = pairing_saved_ = false;
        if (paired_ != nullptr)
          paired_->publish_state(false);
      }
      begin_exchange_(enrollment_.outgoing());
    } else {
      schedule_();
    }
    return;
  }
  if (stage_ == Stage::WRITE) {
    if (!saw_ack_ && attempts_ < 5) {
      schedule_();
      return;
    }
    all_write_acks_ &= saw_ack_;
    if (++write_index_ < write_count_) {
      begin_exchange_(writes_[write_index_]);
      return;
    }
    if (acknowledged_ != nullptr)
      acknowledged_->publish_state(all_write_acks_ && !have_queued_control_);
    stage_ = Stage::VERIFY;
    settle_started_ms_ = millis();
    report_("Writes sent; checking independent status (ACK is not readback)");
    begin_exchange_(status_query(network_, gateway_));
    return;
  }
  if (actual_status(received_status_)) {
    publish_status_(received_status_);
    if (stage_ == Stage::PAIR_STATUS) {
      report_pairing_(pairing_saved_ ? "Paired: registry saved and actual HVAC status verified"
                                     : "Paired: registry and actual HVAC status verified; NOT saved");
      finish_("Gateway pairing and fresh actual climate status verified", true);
      return;
    }
    if (stage_ == Stage::BEFORE) {
      if (complete_association_) {
        complete_association_ = false;
        stage_ = Stage::PAIRING;
        transaction_started_ms_ = millis();
        enrollment_.start_commit(network_, gateway_);
        report_pairing_stage_();
        begin_exchange_(enrollment_.outgoing());
        return;
      }
      if (!prepare_controls(received_status_, requested_, network_, gateway_, writes_, write_count_, expected_)) {
        finish_("Rejected: unsupported state, temperature range, hold, or deadband", false);
        return;
      }
      if (write_count_ == 0) {
        if (acknowledged_ != nullptr)
          acknowledged_->publish_state(saw_ack_);
        finish_("Fresh status received; no write requested", true);
      } else {
        stage_ = Stage::WRITE;
        report_("Fresh state acquired; sending requested controls");
        begin_exchange_(writes_[0]);
      }
      return;
    }
    if (matches(received_status_, expected_)) {
      finish_("Requested state independently verified", true);
      return;
    }
  } else if (attempts_ < 5) {
    schedule_();
    return;
  }
  if (settle_started_ms_ == 0)
    settle_started_ms_ = millis();
  const uint32_t budget = stage_ == Stage::BEFORE ? 20000U : ACTUAL_STATUS_TIMEOUT_MS;
  if (millis() - settle_started_ms_ >= budget) {
    finish_(stage_ == Stage::BEFORE ? "Fresh status unavailable; no write sent"
                                    : "Readback unverified; transmitted writes may have applied",
            false);
    return;
  }
  retry_after_ms_ = millis() + 1000;
  exchange_ = Exchange::SETTLE;
}

void RedLinkClimate::loop() {
  if (this->is_failed()) {
    if (stage_ != Stage::IDLE)
      finish_("Radio failed; transmitted packets may have applied", false);
    return;
  }
  // Finish an on-air packet even if the enable switch was just turned off.
  if (exchange_ == Exchange::TX) {
    if (!tx_finished()) {
      if (millis() - tx_started_ms_ < 50)
        return;
      begin_rx();
      finish_("TX timeout; delivery unknown", false);
      exchange_ = Exchange::CLOCK;
      return;
    }
    const bool success = tx_succeeded() && !this->is_failed();
    if (stage_ == Stage::PAIRING)
      enrollment_.transport_complete(success);
    begin_rx();
    if (!success) {
      if (stage_ != Stage::IDLE)
        finish_("TX failed; delivery unknown; no further packets sent", false);
      exchange_ = Exchange::CLOCK;
      return;
    }
    window_started_ms_ = millis();
    exchange_ = Exchange::RX;
    return;
  }
  Frame frame;
  uint32_t sync_us;
  if (read_frame(frame, sync_us))
    received_(frame, sync_us);
  const bool locked = fresh_clock_();
  if (locked != clock_published_) {
    clock_published_ = locked;
    if (clock_locked_ != nullptr)
      clock_locked_->publish_state(locked);
  }
  if (stage_ == Stage::IDLE || exchange_ == Exchange::CLOCK || exchange_ == Exchange::SETTLE)
    listen_for_clock_();
  if (stage_ != Stage::IDLE && !tx_allowed_()) {
    finish_("TX authorization removed; no further packets sent", false);
    return;
  }
  if (stage_ == Stage::IDLE) {
    if (have_queued_control_ && tx_allowed_()) {
      const Control next = queued_control_;
      have_queued_control_ = false;
      start_(next);
    } else if (tx_allowed_() && millis() - last_refresh_ms_ >= 60000)
      refresh();
    return;
  }
  if (millis() - transaction_started_ms_ >= (stage_ == Stage::PAIRING ? 120000U : 240000U)) {
    finish_("Transaction timeout; check readback before assuming a write failed", false);
    return;
  }
  if ((stage_ == Stage::VERIFY || stage_ == Stage::PAIR_STATUS) &&
      millis() - settle_started_ms_ >= ACTUAL_STATUS_TIMEOUT_MS) {
    finish_("Actual status timeout; transmitted writes may have applied; not resending writes", false);
    return;
  }
  switch (exchange_) {
    case Exchange::CLOCK:
      if (locked)
        schedule_();
      else if (millis() - clock_wait_started_ms_ >= 20000)
        finish_("No fresh EIM clock; no further packets sent", false);
      break;
    case Exchange::SLOT: {
      if (!locked) {
        schedule_();
        break;
      }
      int32_t remaining = static_cast<int32_t>(due_us_ - micros());
      if (remaining > 1000)
        break;
      if (remaining < -250) {
        schedule_();
        break;
      }  // Missed a slot; never send late.
      if (remaining > 0)
        delay_microseconds_safe(remaining);
      // Recheck immediately before transport; neither stale clock nor a disabled switch can reach STX.
      if (!tx_allowed_() || !fresh_clock_()) {
        schedule_();
        break;
      }
      if (stage_ == Stage::PAIRING)
        enrollment_.transmitted();
      if (!send_frame(outgoing_)) {
        ESP_LOGI(TAG, "Trace t=%lu TX refused type=0x%02x stage=%u attempt=%u elapsed=%lu ms", (unsigned long) millis(),
                 outgoing_.data[1], unsigned(stage_), unsigned(attempts_ + 1),
                 (unsigned long) (millis() - transaction_started_ms_));
        finish_("Radio refused TX; delivery unknown", false);
        break;
      }
      last_tx_us_ = micros();
      sent_any_ = true;
      attempts_++;
      saw_ack_ = false;
      received_status_ = Status{};
      tx_started_ms_ = millis();
      exchange_ = Exchange::TX;
      ESP_LOGI(TAG, "Trace t=%lu TX type=0x%02x stage=%u attempt=%u write=%u/%u elapsed=%lu ms",
               (unsigned long) tx_started_ms_, outgoing_.data[1], unsigned(stage_), unsigned(attempts_),
               unsigned(write_index_ + 1), unsigned(write_count_),
               (unsigned long) (tx_started_ms_ - transaction_started_ms_));
      break;
    }
    case Exchange::RX:
      if (millis() - window_started_ms_ >= (stage_ == Stage::WRITE ? 150U : 400U))
        finish_window_();
      break;
    case Exchange::SETTLE:
      if (static_cast<int32_t>(millis() - retry_after_ms_) >= 0)
        begin_exchange_(status_query(network_, gateway_));
      break;
    case Exchange::TX:
      break;
  }
}

}  // namespace esphome::redlink
