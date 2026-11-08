#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace esphome {
namespace redlink {

struct Frame {
  // One-byte payload length plus length/CRC; RX streams across the 64-byte hardware FIFO.
  std::array<uint8_t, 258> data{};
  size_t size{0};
};

struct Status {
  std::array<uint8_t, 31> thermostat{};
  std::array<uint8_t, 10> fan{};
  bool have_thermostat{false};
  bool have_fan{false};
  bool complete() const { return have_thermostat && have_fan; }
};

struct Control {
  int heat{-1}, cool{-1}, mode{-1}, fan{-1}, hold{-1}, until{-1};
  void merge(const Control &newer) {
    if (newer.heat != -1)
      heat = newer.heat;
    if (newer.cool != -1)
      cool = newer.cool;
    if (newer.mode != -1)
      mode = newer.mode;
    if (newer.fan != -1)
      fan = newer.fan;
    if (newer.hold != -1) {
      hold = newer.hold;
      until = newer.until;  // A newer hold selection replaces its deadline too.
    } else if (newer.until != -1)
      until = newer.until;
  }
};

uint16_t crc16(const uint8_t *data, size_t size);
void whiten(uint8_t *data, size_t size);
bool valid_frame(const Frame &frame);
Frame status_query(uint16_t network, uint16_t gateway);
Frame status_reply_ack(uint16_t network, uint16_t gateway);
Frame gateway_discovery(uint16_t gateway);
Frame gateway_announcement(uint16_t gateway);
Frame gateway_registration(uint16_t gateway);
Frame gateway_commit(uint16_t network, uint16_t gateway);
Frame gateway_registry_query(uint16_t network, uint16_t gateway);
bool peer_reply(const Frame &frame, uint16_t network, uint16_t gateway);
bool acknowledgement(const Frame &frame, uint16_t network, uint16_t gateway);
bool discovery_reply(const Frame &frame, uint16_t network, uint16_t gateway);
bool registration_reply(const Frame &frame, uint16_t network, uint16_t gateway);
bool registry_reply(const Frame &frame, uint16_t network, uint16_t gateway);
// A malformed frame never changes accumulated status. Address checking belongs to the caller.
bool collect_status(const Frame &frame, Status &status);
bool actual_status(const Status &status);
// The caller must obtain fresh status before preparing each control transaction.
// Failed preparation leaves all outputs unchanged; -1 means unrequested.
bool prepare_controls(const Status &status, const Control &control, uint16_t network, uint16_t gateway,
                      std::array<Frame, 2> &frames, size_t &count, Status &expected);
bool matches(const Status &actual, const Status &expected);
struct HopChannel {
  uint8_t channel;
  uint16_t sync;
};
// Physical candidates in frequency order, NOT an installation's logical hop order.
constexpr size_t HOP_COUNT = 50;
constexpr uint8_t UNKNOWN_CHANNEL = 0xff;
extern const std::array<uint8_t, HOP_COUNT> CANDIDATE_CHANNELS;
HopChannel candidate_channel(uint8_t rank);

// Fixed byte layout: magic/version/profile, network, tuning reference, slot ranks, CRC.
struct HopMapRecord {
  std::array<uint8_t, 62> bytes{};
};

class HopLearner {
 public:
  HopLearner() { reset(); }
  void reset(bool retain_active = false);
  void restart_clock_epoch();
  void reject_conflict() { conflict_ = true; }
  bool observe(uint8_t slot, uint8_t rank, uint32_t counter, uint32_t observed_ms = 0);
  bool ready() const { return active_valid_ && !rebuilding_ && !conflict_ && fresh_count_ >= 2; }
  bool conflicted() const { return conflict_; }
  bool cached() const { return active_valid_ && !rebuilding_; }
  uint8_t known() const;
  uint8_t confirmed() const;
  uint8_t rank(uint8_t slot) const;
  uint8_t active_rank(uint8_t slot) const { return active_[slot]; }
  bool load(const HopMapRecord &record, uint16_t network, uint32_t frequency);
  bool save(HopMapRecord &record, uint16_t network, uint32_t frequency) const;

 protected:
  std::array<uint8_t, HOP_COUNT> pending_{}, observations_{}, active_{};
  std::array<uint32_t, HOP_COUNT> counters_{};
  std::array<uint32_t, HOP_COUNT> observed_ms_{};
  bool active_valid_{false}, rebuilding_{true}, conflict_{false};
  uint8_t fresh_count_{0};
  uint32_t fresh_counter_{0};
};
struct ClockListenWindow {
  uint8_t slot{0}, copy{0};
  uint32_t sync_us{0};
  void advance();
};
bool clock_slot(const Frame &frame, uint16_t network, uint8_t &slot, uint32_t &phase_ticks, ClockListenWindow &next,
                uint32_t *counter = nullptr);

// One explicitly requested enrollment. Transport and the slot clock remain the caller's responsibility.
// Registration and completion need exact readback; a commit ACK only advances to registry verification.
class GatewayEnrollment {
 public:
  enum class State { IDLE, DISCOVER, ANNOUNCE, REGISTER, COMMIT, VERIFY, COMPLETE, FAILED };
  void start(uint16_t network, uint16_t gateway);
  void start_commit(uint16_t network, uint16_t gateway);
  Frame outgoing() const;
  void transmitted();
  void transport_complete(bool success);
  void receive(const Frame &frame);
  void finish_window();
  void fail() { state_ = State::FAILED; }
  State state() const { return state_; }
  bool may_have_changed() const { return may_have_changed_; }
  bool verified() const { return state_ == State::COMPLETE; }

 protected:
  void advance_(State state);
  State state_{State::IDLE};
  uint16_t network_{0}, gateway_{0};
  uint8_t attempts_{0};
  bool transport_ok_{false}, response_{false}, may_have_changed_{false};
};

}  // namespace redlink
}  // namespace esphome
