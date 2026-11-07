#include "protocol.h"

#include <algorithm>

namespace esphome {
namespace redlink {

const std::array<uint8_t, HOP_COUNT> CANDIDATE_CHANNELS{
    4,  6,  8,  10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 44, 46, 48,  50,  52,  54,  56,  58,  60,
    62, 66, 68, 70, 73, 75, 77, 79, 81, 83, 85, 87, 89, 91, 93, 95, 97, 99, 109, 111, 113, 115, 117, 119, 121};

HopChannel candidate_channel(uint8_t rank) {
  if (rank >= HOP_COUNT)
    return {UNKNOWN_CHANNEL, 0};
  return {CANDIDATE_CHANNELS[rank], static_cast<uint16_t>(0x636b + rank)};
}

void HopLearner::reset(bool retain_active) {
  pending_.fill(UNKNOWN_CHANNEL);
  observations_.fill(0);
  counters_.fill(0x1000000);  // Outside the 24-bit counter domain: no observation this epoch.
  if (!retain_active) {
    active_.fill(UNKNOWN_CHANNEL);
    active_valid_ = false;
  }
  rebuilding_ = true;
  conflict_ = false;
  fresh_count_ = 0;
  fresh_counter_ = 0;
}

void HopLearner::restart_clock_epoch() {
  // A reset/outage invalidates counter ordering, not an already confirmed map.
  // Single observations lose credit, so replaying one old packet cannot confirm it.
  counters_.fill(0x1000000);
  for (auto &count : observations_)
    if (count < 2)
      count = 0;
  fresh_count_ = 0;
  fresh_counter_ = 0;
}

uint8_t HopLearner::known() const {
  return cached()
             ? HOP_COUNT
             : std::count_if(pending_.begin(), pending_.end(), [](uint8_t rank) { return rank != UNKNOWN_CHANNEL; });
}

uint8_t HopLearner::confirmed() const {
  return cached() && ready() ? HOP_COUNT : std::count(observations_.begin(), observations_.end(), uint8_t(2));
}

uint8_t HopLearner::rank(uint8_t slot) const {
  if (slot >= HOP_COUNT)
    return UNKNOWN_CHANNEL;
  return cached() ? active_[slot] : pending_[slot];
}

bool HopLearner::observe(uint8_t slot, uint8_t rank, uint32_t counter, uint32_t observed_ms) {
  if (slot >= HOP_COUNT || rank >= HOP_COUNT || counter > 0xffffff || conflict_)
    return false;
  if ((cached() && active_[slot] != rank) || (pending_[slot] != UNKNOWN_CHANNEL && pending_[slot] != rank)) {
    conflict_ = true;
    return false;
  }
  for (size_t i = 0; i < HOP_COUNT; ++i) {
    // Ordering is ambiguous after half of the 24-bit/4096-Hz counter period.
    if (counters_[i] <= 0xffffff && observed_ms - observed_ms_[i] >= 2048000) {
      counters_[i] = 0x1000000;
      if (observations_[i] < 2)
        observations_[i] = 0;
    }
    if (i != slot && (pending_[i] == rank || counters_[i] == counter)) {
      conflict_ = true;
      return false;
    }
  }
  // Live reception is ordered. Repeated or older counters never add evidence.
  const uint32_t delta = (counter - counters_[slot]) & 0xffffff;
  if (counters_[slot] <= 0xffffff && (delta == 0 || delta >= 0x800000))
    return false;
  pending_[slot] = rank;
  counters_[slot] = counter;
  observed_ms_[slot] = observed_ms;
  if (observations_[slot] < 2)
    ++observations_[slot];
  if (cached()) {
    const uint32_t fresh_delta = (counter - fresh_counter_) & 0xffffff;
    if (fresh_count_ == 0 || (fresh_delta != 0 && fresh_delta < 0x800000)) {
      fresh_counter_ = counter;
      if (fresh_count_ < 2)
        ++fresh_count_;
    }
  } else if (confirmed() == HOP_COUNT) {
    active_ = pending_;
    active_valid_ = true;
    rebuilding_ = false;
    fresh_count_ = 2;
    fresh_counter_ = counter;
  }
  return true;
}

bool HopLearner::load(const HopMapRecord &record, uint16_t network, uint32_t frequency) {
  const auto &b = record.bytes;
  if (network == 0 || network == 0xffff || b[0] != 0x52 || b[1] != 0x4c || b[2] != 1 || b[3] != 1 ||
      ((uint16_t(b[4]) << 8) | b[5]) != network ||
      ((uint32_t(b[6]) << 24) | (uint32_t(b[7]) << 16) | (uint32_t(b[8]) << 8) | b[9]) != frequency ||
      crc16(b.data(), 60) != ((uint16_t(b[60]) << 8) | b[61]))
    return false;
  std::array<bool, HOP_COUNT> seen{};
  for (size_t i = 0; i < HOP_COUNT; ++i) {
    if (b[10 + i] >= HOP_COUNT || seen[b[10 + i]])
      return false;
    seen[b[10 + i]] = true;
  }
  reset();
  std::copy_n(b.begin() + 10, HOP_COUNT, active_.begin());
  active_valid_ = true;
  rebuilding_ = false;
  return true;  // Still TX-ineligible until independent fresh clock traffic agrees.
}

bool HopLearner::save(HopMapRecord &record, uint16_t network, uint32_t frequency) const {
  if (!ready() || network == 0 || network == 0xffff)
    return false;
  auto &b = record.bytes;
  b[0] = 0x52;
  b[1] = 0x4c;
  b[2] = 1;  // Storage version.
  b[3] = 1;  // Candidate/sync/timing profile version.
  b[4] = network >> 8;
  b[5] = network;
  b[6] = frequency >> 24;
  b[7] = frequency >> 16;
  b[8] = frequency >> 8;
  b[9] = frequency;
  std::copy(active_.begin(), active_.end(), b.begin() + 10);
  const uint16_t crc = crc16(b.data(), 60);
  b[60] = crc >> 8;
  b[61] = crc;
  return true;
}
namespace {

constexpr std::array<uint8_t, 31> THERMOSTAT_WRITE{0x1e, 0x81, 0xff, 0x00, 0x9a, 0x80, 0x80, 0x46, 0x4a, 0x2f, 0xff,
                                                   0x5c, 0xdc, 0x80, 0x80, 0x80, 0x80, 0x46, 0x4a, 0xfc, 0x02, 0x80,
                                                   0x2f, 0x84, 0x80, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00};
constexpr std::array<uint8_t, 10> FAN_WRITE{0x09, 0xa2, 0xc0, 0x00, 0x85, 0x01, 0x20, 0x01, 0x00, 0x0d};
constexpr uint32_t COUNTER_MASK = 0xffffff;

uint16_t read16(const uint8_t *data) { return (uint16_t(data[0]) << 8) | data[1]; }
uint32_t read24(const uint8_t *data) { return (uint32_t(data[0]) << 16) | (uint32_t(data[1]) << 8) | data[2]; }

void finish_frame(Frame &frame) {
  frame.data[0] = static_cast<uint8_t>(frame.size - 3);
  const uint16_t crc = crc16(frame.data.data(), frame.size - 2);
  frame.data[frame.size - 2] = static_cast<uint8_t>(crc >> 8);
  frame.data[frame.size - 1] = static_cast<uint8_t>(crc);
}

Frame addressed_frame(uint8_t type, uint16_t network, uint16_t gateway) {
  Frame frame;
  frame.data[1] = type;
  frame.data[2] = frame.data[3] = 0xff;
  frame.data[4] = static_cast<uint8_t>(network >> 8);
  frame.data[5] = static_cast<uint8_t>(network);
  frame.data[6] = static_cast<uint8_t>(gateway >> 8);
  frame.data[7] = static_cast<uint8_t>(gateway);
  frame.data[8] = 0xf0;
  return frame;
}

template<size_t N>
Frame block_frame(uint8_t type, const std::array<uint8_t, N> &block, uint16_t network, uint16_t gateway) {
  static_assert(N + 13 <= 64, "Parameter block exceeds frame capacity");
  Frame frame = addressed_frame(type, network, gateway);
  std::copy(block.begin(), block.end(), frame.data.begin() + 11);
  frame.size = N + 13;
  finish_frame(frame);
  return frame;
}

template<size_t N> Frame write_frame(const std::array<uint8_t, N> &block, uint16_t network, uint16_t gateway) {
  return block_frame(0x00, block, network, gateway);
}

bool target_valid(int heat, int cool) {
  return heat >= 55 && heat <= 80 && cool >= 55 && cool <= 80 && cool - heat >= 2;
}
bool fan_valid(int fan) { return fan == 0 || fan == 1 || fan == 3; }
int heat_hold(const Status &status) { return (status.thermostat[9] >> 3) & 3; }
int cool_hold(const Status &status) { return (status.thermostat[9] >> 5) & 3; }
bool deadlines_valid(const Status &status) {
  return (heat_hold(status) != 1 || (status.thermostat[11] & 0x7f) < 96) &&
         (cool_hold(status) != 1 || (status.thermostat[12] & 0x7f) < 96);
}
bool supported_state(const Status &status) {
  const int mode = status.thermostat[19] & 7;
  return status.complete() && status.thermostat[4] == 0x5a && status.fan[4] == 0x45 && mode <= 4 &&
         fan_valid(status.fan[9] & 3) && heat_hold(status) <= 2 && cool_hold(status) <= 2 && deadlines_valid(status);
}

}  // namespace

uint16_t crc16(const uint8_t *data, size_t size) {
  uint16_t crc = 0xffff;
  for (size_t i = 0; i < size; i++) {
    crc ^= uint16_t(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; bit++)
      crc = static_cast<uint16_t>((crc << 1) ^ ((crc & 0x8000) ? 0x8005 : 0));
  }
  return crc;
}

void whiten(uint8_t *data, size_t size) {
  uint16_t state = 0x1ff;
  for (size_t i = 0; i < size; i++) {
    data[i] ^= static_cast<uint8_t>(state);
    for (uint8_t bit = 0; bit < 8; bit++)
      state = (state >> 1) | (((state ^ (state >> 5)) & 1) << 8);
  }
}

bool valid_frame(const Frame &frame) {
  return frame.size >= 13 && frame.size <= frame.data.size() && frame.size == size_t(frame.data[0]) + 3 &&
         crc16(frame.data.data(), frame.size - 2) == read16(frame.data.data() + frame.size - 2);
}

Frame status_query(uint16_t network, uint16_t gateway) {
  Frame frame = addressed_frame(0x10, network, gateway);
  constexpr std::array<uint8_t, 11> BLOCKS{0x04, 0x01, 0xff, 0x00, 0x00, 0x00, 0x04, 0x22, 0xc0, 0x00, 0x00};
  std::copy(BLOCKS.begin(), BLOCKS.end(), frame.data.begin() + 11);
  frame.size = BLOCKS.size() + 13;
  finish_frame(frame);
  return frame;
}

Frame gateway_discovery(uint16_t gateway) {
  return block_frame(0x33, std::array<uint8_t, 2>{0x81, 0x11}, 0xfffe, gateway);
}

Frame gateway_announcement(uint16_t gateway) {
  return block_frame(0x20, std::array<uint8_t, 2>{0x81, 0x60}, 0xfffe, gateway);
}

Frame gateway_registration(uint16_t gateway) {
  return block_frame(0x30,
                     std::array<uint8_t, 8>{0x87, 0x82, 0x03, 0x00, static_cast<uint8_t>(gateway >> 8),
                                            static_cast<uint8_t>(gateway), 0xf0, 0xd2},
                     0xfffe, gateway);
}

Frame gateway_commit(uint16_t network, uint16_t gateway) {
  return write_frame(std::array<uint8_t, 8>{0x07, 0x12, 0x50, 0x00, 0x43, 0x00, static_cast<uint8_t>(gateway >> 8),
                                            static_cast<uint8_t>(gateway)},
                     network, gateway);
}

Frame gateway_registry_query(uint16_t network, uint16_t gateway) {
  return block_frame(0x10, std::array<uint8_t, 5>{0x04, 0x12, 0x50, 0x00, 0x00}, network, gateway);
}

bool peer_reply(const Frame &frame, uint16_t network, uint16_t gateway) {
  return valid_frame(frame) && read16(frame.data.data() + 2) == 0xffff && read16(frame.data.data() + 4) == gateway &&
         read16(frame.data.data() + 6) == network && frame.data[8] == 0x00 && frame.data[9] == 0x00 &&
         frame.data[10] == 0xf0;
}

bool acknowledgement(const Frame &frame, uint16_t network, uint16_t gateway) {
  return peer_reply(frame, network, gateway) && frame.data[1] == 0x22 && frame.size == 16 && frame.data[11] == 0x82 &&
         frame.data[12] == 0x52;
}

bool discovery_reply(const Frame &frame, uint16_t network, uint16_t gateway) {
  return peer_reply(frame, network, gateway) && (frame.data[1] == 0x2a || frame.data[1] == 0x2e) && frame.size == 24 &&
         frame.data[11] == 0x8a && frame.data[12] == 0x51;
}

bool registration_reply(const Frame &frame, uint16_t network, uint16_t gateway) {
  return peer_reply(frame, network, gateway) && frame.data[1] == 0x28 && frame.size == 23 && frame.data[11] == 0x89 &&
         frame.data[12] == 0x44 && read16(frame.data.data() + 13) == network && frame.data[15] == 0xf0 &&
         read16(frame.data.data() + 16) == gateway && frame.data[18] == 0x01 && frame.data[19] == 0x00 &&
         frame.data[20] == 0x80;
}

bool registry_reply(const Frame &frame, uint16_t network, uint16_t gateway) {
  return peer_reply(frame, network, gateway) && frame.data[1] == 0x28 && frame.size == 21 && frame.data[11] == 0x07 &&
         frame.data[12] == 0x12 && frame.data[13] == 0x50 && frame.data[14] == 0x00 && frame.data[15] == 0x43 &&
         frame.data[16] == 0x00 && read16(frame.data.data() + 17) == gateway;
}

bool collect_status(const Frame &frame, Status &status) {
  if (!valid_frame(frame) || (frame.data[1] != 0x28 && frame.data[1] != 0x2c))
    return false;
  Status candidate = status;
  bool found = false;
  const size_t end = frame.size - 2;
  size_t position = 11;
  while (position < end) {
    const uint8_t *block = frame.data.data() + position;
    const size_t size = (block[0] & 0x7f) + 1;
    if (size > end - position)
      return false;
    if (size == 31 && (block[1] & 0x3f) == 1 && block[2] == 0xff && block[3] == 0) {
      if (block[4] == 0x5a) {
        std::copy_n(block, candidate.thermostat.size(), candidate.thermostat.begin());
        candidate.have_thermostat = true;
        found = true;
      } else if (block[4] == 0x9a) {
        candidate.have_thermostat = false;
        found = true;
      }
    } else if (size == 10 && (block[1] & 0x3f) == 0x22 && block[2] == 0xc0 && block[3] == 0) {
      if (block[4] == 0x45) {
        std::copy_n(block, candidate.fan.size(), candidate.fan.begin());
        candidate.have_fan = true;
        found = true;
      } else if (block[4] == 0x85) {
        candidate.have_fan = false;
        found = true;
      }
    }
    position += size;
    if (position < end) {
      if (frame.data[position] != frame.data[10] || ++position == end)
        return false;
    }
  }
  // Commit only after every block and separator has passed validation.
  if (found)
    status = candidate;
  return found;
}

bool actual_status(const Status &status) { return supported_state(status); }

bool prepare_controls(const Status &status, const Control &control, uint16_t network, uint16_t gateway,
                      std::array<Frame, 2> &frames, size_t &count, Status &expected) {
  if (!supported_state(status) || (control.heat != -1 && (control.heat < 55 || control.heat > 80)) ||
      (control.cool != -1 && (control.cool < 55 || control.cool > 80)) ||
      (control.mode != -1 && (control.mode < 0 || control.mode > 4)) ||
      (control.fan != -1 && !fan_valid(control.fan)) || control.hold < -1 || control.hold > 2 || control.until < -1 ||
      control.until > 95 || (control.until != -1 && control.hold != 1) ||
      (control.hold == 0 && (control.heat != -1 || control.cool != -1)))
    return false;

  Status candidate = status;
  std::array<Frame, 2> prepared{};
  size_t prepared_count = 0;
  if (control.heat != -1 || control.cool != -1 || control.mode != -1 || control.hold != -1) {
    auto command = THERMOSTAT_WRITE;
    // These writable context bytes must come from this transaction's fresh EIM status.
    constexpr std::array<size_t, 11> CONTEXT{7, 8, 9, 11, 12, 17, 18, 19, 20, 22, 23};
    for (const size_t index : CONTEXT)
      command[index] = status.thermostat[index];
    command[9] |= 1;
    if (control.heat != -1)
      command[7] = static_cast<uint8_t>(control.heat);
    if (control.cool != -1)
      command[8] = static_cast<uint8_t>(control.cool);
    if (control.mode != -1)
      command[19] = (command[19] & 0xf8) | control.mode;
    int hold = control.hold;
    if ((control.heat != -1 || control.cool != -1) && hold == -1 && heat_hold(status) == 0 && cool_hold(status) == 0)
      hold = 1;
    if (hold != -1) {
      command[9] = (command[9] & ~0x78) | (hold << 3) | (hold << 5);
      if (hold == 0) {
        command[7] = status.thermostat[17];
        command[8] = status.thermostat[18];
      }
    }
    if (control.until != -1) {
      command[11] = (command[11] & 0x80) | control.until;
      command[12] = (command[12] & 0x80) | control.until;
    }
    if (!target_valid(command[7], command[8]))
      return false;
    candidate.thermostat = command;
    if (!deadlines_valid(candidate))
      return false;
    if (!matches(status, candidate))
      prepared[prepared_count++] = write_frame(command, network, gateway);
  }
  // A fan-only request never emits a thermostat write or changes its expected state.
  if (control.fan != -1 && (status.fan[9] & 3) != control.fan) {
    auto command = FAN_WRITE;
    command[5] = control.fan == 1 ? 1 : 0;
    command[6] = control.fan == 3 ? 0xa0 : 0x20;
    command[9] = 0x0c | control.fan;
    candidate.fan = command;
    prepared[prepared_count++] = write_frame(command, network, gateway);
  }
  frames = prepared;
  count = prepared_count;
  expected = candidate;
  return true;
}

bool matches(const Status &actual, const Status &expected) {
  if (!supported_state(actual) || !expected.complete() || !deadlines_valid(expected))
    return false;
  const auto &a = actual.thermostat;
  const auto &e = expected.thermostat;
  // State.values() exposes semantic controls, not write/status flags or live measurements.
  const auto mode = [](uint8_t value) {
    const int code = value & 7;
    return code <= 4 ? code : -1;
  };
  return a[7] == e[7] && a[8] == e[8] && mode(a[19]) == mode(e[19]) && (actual.fan[9] & 3) == (expected.fan[9] & 3) &&
         heat_hold(actual) == heat_hold(expected) && cool_hold(actual) == cool_hold(expected) && a[17] == e[17] &&
         a[18] == e[18] && (heat_hold(actual) != 1 || (a[11] & 0x7f) == (e[11] & 0x7f)) &&
         (cool_hold(actual) != 1 || (a[12] & 0x7f) == (e[12] & 0x7f));
}

void ClockListenWindow::advance() {
  const uint32_t ticks = copy < 3 ? 242 : 61611 - 3 * 242;
  slot = static_cast<uint8_t>((slot + (copy < 3 ? 1 : 48)) % 50);
  copy = (copy + 1) % 4;
  sync_us += static_cast<uint32_t>((uint64_t(ticks) * 1000000 + 2048) / 4096);
}

bool clock_slot(const Frame &frame, uint16_t network, uint8_t &slot, uint32_t &phase_ticks, ClockListenWindow &next,
                uint32_t *counter) {
  if (!valid_frame(frame) || (frame.data[1] != 0x02 && frame.data[1] != 0x06) ||
      read16(frame.data.data() + 2) != 0xffff || read16(frame.data.data() + 4) != network ||
      read16(frame.data.data() + 6) != network || frame.data[8] != 0 || frame.data[9] != 0 || frame.data[10] != 0xff)
    return false;
  bool found = false;
  uint32_t c1 = 0, c2 = 0, c3 = 0;
  size_t position = 11;
  const size_t end = frame.size - 2;
  while (position < end) {
    const uint8_t marker = frame.data[position++];
    if (marker == 0xff)
      continue;
    const size_t length = marker & 0x7f;
    if (length > end - position)
      return false;
    if (marker == 0x8a) {
      if (found || frame.data[position] != 0x51)
        return false;
      c1 = read24(frame.data.data() + position + 1);
      c2 = read24(frame.data.data() + position + 4);
      c3 = read24(frame.data.data() + position + 7);
      found = true;
    }
    position += length;
  }
  if (!found)
    return false;
  // Counter wrap is valid; relationships are measured in the 24-bit clock domain.
  const uint32_t delta = (c1 - c2) & COUNTER_MASK;
  const uint32_t base = (c2 - c3) & COUNTER_MASK;
  if (delta < 18 || delta > 744 || (delta - 18) % 242 != 0 || base >= 550 || base % 11 != 0)
    return false;
  const uint8_t copy = (delta - 18) / 242;
  slot = static_cast<uint8_t>((base / 11 + copy) % 50);
  phase_ticks = (c1 - c3) & COUNTER_MASK;
  next = {slot, copy, 0};
  next.advance();
  if (counter != nullptr)
    *counter = c1;
  return true;
}

void GatewayEnrollment::advance_(State state) {
  state_ = state;
  attempts_ = 0;
  transport_ok_ = response_ = false;
}

void GatewayEnrollment::start(uint16_t network, uint16_t gateway) {
  network_ = network;
  gateway_ = gateway;
  may_have_changed_ = false;
  advance_(State::DISCOVER);
}

void GatewayEnrollment::start_commit(uint16_t network, uint16_t gateway) {
  start(network, gateway);
  advance_(State::COMMIT);
}

Frame GatewayEnrollment::outgoing() const {
  switch (state_) {
    case State::DISCOVER:
      return gateway_discovery(gateway_);
    case State::ANNOUNCE:
      return gateway_announcement(gateway_);
    case State::REGISTER:
      return gateway_registration(gateway_);
    case State::COMMIT:
      return gateway_commit(network_, gateway_);
    case State::VERIFY:
      return gateway_registry_query(network_, gateway_);
    default:
      return Frame{};
  }
}

void GatewayEnrollment::transmitted() {
  if (state_ == State::IDLE || state_ == State::COMPLETE || state_ == State::FAILED)
    return;
  ++attempts_;
  transport_ok_ = response_ = false;
  if (state_ == State::REGISTER || state_ == State::COMMIT)
    may_have_changed_ = true;
}

void GatewayEnrollment::transport_complete(bool success) {
  if (state_ == State::IDLE || state_ == State::COMPLETE || state_ == State::FAILED)
    return;
  if (!success || attempts_ == 0) {
    fail();
    return;
  }
  transport_ok_ = true;
}

void GatewayEnrollment::receive(const Frame &frame) {
  if (!transport_ok_)
    return;
  if (state_ == State::DISCOVER)
    response_ |= discovery_reply(frame, network_, gateway_);
  else if (state_ == State::REGISTER)
    response_ |= registration_reply(frame, network_, gateway_);
  else if (state_ == State::COMMIT)
    response_ |= acknowledgement(frame, network_, gateway_);
  else if (state_ == State::VERIFY)
    response_ |= registry_reply(frame, network_, gateway_);
}

void GatewayEnrollment::finish_window() {
  if (state_ == State::IDLE || state_ == State::COMPLETE || state_ == State::FAILED)
    return;
  if (!transport_ok_) {
    fail();
    return;
  }
  if (state_ == State::ANNOUNCE) {
    advance_(State::REGISTER);
    return;
  }
  if (state_ == State::COMMIT) {
    // A drained TX FIFO is not delivery. Retry this idempotent association write
    // within the normal five-attempt bound, then always check the registry.
    if (response_ || attempts_ >= 5)
      advance_(State::VERIFY);
    return;
  }
  if (!response_) {
    if (attempts_ >= 5)
      fail();
    return;
  }
  if (state_ == State::DISCOVER)
    advance_(State::ANNOUNCE);
  else if (state_ == State::REGISTER)
    advance_(State::COMMIT);
  else if (state_ == State::VERIFY)
    advance_(State::COMPLETE);
}

}  // namespace redlink
}  // namespace esphome
