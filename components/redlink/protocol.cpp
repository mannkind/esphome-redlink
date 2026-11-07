#include "protocol.h"

#include <algorithm>

namespace esphome {
namespace redlink {

// Fixed experimental hop/sync profile measured on installation network 0x389e.
// This is not a universal RedLINK map: changing network_id does not discover or
// derive another installation's profile. Slots 41/43/44 were confirmed by
// CRC-valid gateway-pairing captures; the remaining entries retain the measured profile.
const std::array<HopChannel, 50> HOP_CHANNELS{{
    {10, 0x636e},  {70, 0x6387},  {50, 0x637e},  {111, 0x6397}, {26, 0x6376},  {87, 0x638f},  {58, 0x6382},
    {119, 0x639b}, {89, 0x6390},  {28, 0x6377},  {91, 0x6391},  {30, 0x6378},  {77, 0x638a},  {16, 0x6371},
    {14, 0x6370},  {75, 0x6389},  {66, 0x6385},  {6, 0x636c},   {109, 0x6396}, {48, 0x637d},  {73, 0x6388},
    {12, 0x636f},  {113, 0x6398}, {52, 0x637f},  {62, 0x6384},  {4, 0x636b},   {117, 0x639a}, {56, 0x6381},
    {115, 0x6399}, {54, 0x6380},  {32, 0x6379},  {93, 0x6392},  {83, 0x638d},  {22, 0x6374},  {97, 0x6394},
    {44, 0x637b},  {60, 0x6383},  {121, 0x639c}, {99, 0x6395},  {46, 0x637c},  {34, 0x637a},  {95, 0x6393},
    {18, 0x6372},  {79, 0x638b},  {81, 0x638c},  {20, 0x6373},  {24, 0x6375},  {85, 0x638e},  {8, 0x636d},
    {68, 0x6386},
}};
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

bool clock_slot(const Frame &frame, uint16_t network, uint8_t &slot, uint32_t &phase_ticks, ClockListenWindow &next) {
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
  if (!success || attempts_ == 0)
    fail();
  else
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
  if (state_ == State::ANNOUNCE)
    advance_(State::REGISTER);
  else if (state_ == State::COMMIT) {
    // A drained TX FIFO is not delivery. Retry this idempotent association write
    // within the normal five-attempt bound, then always check the registry.
    if (response_ || attempts_ >= 5)
      advance_(State::VERIFY);
  } else if (response_) {
    if (state_ == State::DISCOVER)
      advance_(State::ANNOUNCE);
    else if (state_ == State::REGISTER)
      advance_(State::COMMIT);
    else if (state_ == State::VERIFY)
      advance_(State::COMPLETE);
  } else if (attempts_ >= 5) {
    fail();
  }
}

}  // namespace redlink
}  // namespace esphome
