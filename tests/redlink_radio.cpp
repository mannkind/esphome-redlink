// Compile the actual driver; fake only ESPHome's clock, GPIO, SPI and component surface.
#include "radio.h"
#include "esphome/core/hal.h"

#include "check.h"
#include <algorithm>
#include <cstdio>
#include <functional>
#include <initializer_list>
#include <utility>
#include <vector>

namespace redlink_test {
struct Sample {
  uint8_t value{0};
  uint32_t duration_us{16};  // One 16-clock status transaction at 1 MHz.
};
static uint32_t now;
static uint32_t started;
static std::vector<Sample> samples;
static std::vector<uint32_t> read_starts;
static Sample fallback;
static bool emulate_registers = false, fifo_write = false;
static uint8_t fifo_read_command = 0;
static uint8_t registers[64]{};
static uint8_t marcstate = 1;
static unsigned tx_strobes = 0, rx_strobes = 0, rx_wire_count = 0;
static std::vector<uint8_t> rx_fifo;
static esphome::InternalGPIOPin *gdo = nullptr;
static std::vector<uint8_t> rx_status_samples;
static size_t rx_status_index = 0;
static std::function<void()> after_rx_status, before_idle;
static std::function<void(uint8_t)> before_register_write;
static void advance(uint32_t duration) {
  now += duration;
  // Fail instead of hanging if production loses its bounded timeout.
  CHECK(uint32_t(now - started) <= (emulate_registers ? 200000 : 10000));
}
static void reset(uint32_t start, std::initializer_list<Sample> script = {}, Sample rest = {0x08}) {
  now = started = start;
  samples = script;
  read_starts.clear();
  fallback = rest;
  emulate_registers = fifo_write = false;
  fifo_read_command = 0;
  std::fill_n(registers, 64, 0);
  marcstate = 1;
  tx_strobes = rx_strobes = rx_wire_count = 0;
  rx_fifo.clear();
  gdo = nullptr;
  rx_status_samples.clear();
  rx_status_index = 0;
  after_rx_status = before_idle = {};
  before_register_write = {};
}
// Byte-counter/FIFO model, not a substitute driver. Once fixed PKTLEN is
// reached, later demodulated noise cannot enter the FIFO until another SRX.
static void receive_bytes(const uint8_t *data, size_t size) {
  CHECK(emulate_registers && gdo != nullptr);
  for (size_t i = 0; i < size; ++i) {
    advance(209);  // 8 bits at 38.383 kbit/s.
    if (marcstate != 0x0d || !gdo->high)
      continue;
    if (rx_fifo.size() == 65) {
      marcstate = 0x11;
      registers[0x3b] = 0x80 | 65;
      gdo->high = false;
      continue;
    }
    rx_fifo.push_back(data[i]);
    registers[0x3b] = rx_fifo.size();
    ++rx_wire_count;
    if ((registers[0x08] & 3) == 0 && (rx_wire_count & 0xff) == registers[0x06]) {
      marcstate = 1;
      gdo->high = false;
    }
  }
}
static void receive_noise(size_t size) {
  const uint8_t noise = 0xa5;
  for (size_t i = 0; i < size; ++i)
    receive_bytes(&noise, 1);
}
void transfer_status(uint8_t *data, size_t size) {
  if (emulate_registers) {
    CHECK(size == 2);
    const uint8_t address = data[0] & 0x3f;
    data[1] = address == 0x35 ? marcstate : address == 0x30 ? 0 : address == 0x31 ? 0x14 : registers[address];
    if (address == 0x3b) {
      if (rx_status_index < rx_status_samples.size())
        data[1] = rx_status_samples[rx_status_index++];
      if (after_rx_status) {
        auto hook = std::move(after_rx_status);
        after_rx_status = {};
        hook();
      }
    }
    return;
  }
  CHECK(size == 2 && data[0] == 0xf5 && data[1] == 0);  // Burst-read MARCSTATE only.
  const size_t index = read_starts.size();
  read_starts.push_back(now);
  const Sample sample = index < samples.size() ? samples[index] : fallback;
  advance(sample.duration_us);
  data[1] = sample.value;
}
uint8_t transfer_command(uint8_t command) {
  CHECK(emulate_registers);
  if (command == 0x36) {
    if (before_idle) {
      auto hook = std::move(before_idle);
      before_idle = {};
      hook();
    }
    marcstate = 1;
    if (gdo != nullptr)
      gdo->high = false;
  }
  if (command == 0x34) {
    marcstate = 0x0d;
    rx_wire_count = 0;
    ++rx_strobes;
  }
  if (command == 0x3a) {
    rx_fifo.clear();
    registers[0x3b] = 0;
  }
  if (command == 0x3b)
    registers[0x3a] = 0;
  if (command == 0x35) {
    marcstate = 0x13;
    ++tx_strobes;
  }
  fifo_write = command == 0x7f || command == 0x3f;
  if (command == 0xbf || command == 0xff)
    fifo_read_command = command;
  return 0;
}
void write_bytes(const uint8_t *data, size_t size) {
  CHECK(emulate_registers);
  if (fifo_write) {
    registers[0x3a] = size;
    fifo_write = false;
    return;
  }
  CHECK(size == 2 && data[0] < 64);
  if (before_register_write) {
    auto hook = std::move(before_register_write);
    before_register_write = {};
    hook(data[0]);
  }
  registers[data[0]] = data[1];
}
void read_bytes(uint8_t *data, size_t size) {
  CHECK(emulate_registers && size <= rx_fifo.size() && size <= 64);
  CHECK(fifo_read_command == (size == 1 ? 0xbf : 0xff));
  // Enforce the last-byte duplication workaround at every actual FIFO read.
  CHECK(marcstate != 0x0d || gdo == nullptr || !gdo->high || size < rx_fifo.size());
  fifo_read_command = 0;
  std::copy_n(rx_fifo.begin(), size, data);
  rx_fifo.erase(rx_fifo.begin(), rx_fifo.begin() + size);
  registers[0x3b] = rx_fifo.size();
}
}  // namespace redlink_test

namespace esphome {
uint32_t micros() { return redlink_test::now; }
void delayMicroseconds(uint32_t duration) { redlink_test::advance(duration); }
}  // namespace esphome

class TestRadio : public esphome::redlink::RedLinkRadio {
 public:
  TestRadio() {
    set_miso_pin(&miso);
    set_radio_healthy_sensor(&health);
    health.publish_state(true);
  }
  using RedLinkRadio::wait_state_;
  using RedLinkRadio::listen_candidate_;
  using RedLinkRadio::hop_;
  using RedLinkRadio::receive_candidate_;
  bool receiving() const { return receiving_(); }
  void packet_sync() {
    CHECK(redlink_test::marcstate == 0x0d && !gdo.high);
    gdo.high = true;
    sync_interrupt_(this);
  }
  void delayed_sync(uint32_t age) {
    sync_end_us_ = esphome::micros() - age;
    sync_pending_ = true;
    gdo.high = true;
  }
  void configure_bus() {
    parent_ = this;
    cs_ = &cs;
    set_gdo0_pin(&gdo);
    redlink_test::gdo = &gdo;
  }
  void enable_tx() {
    receive_only_ = false;
    transmit_enabled_ = true;
  }
  esphome::InternalGPIOPin cs, gdo;
  esphome::InternalGPIOPin miso;
  esphome::binary_sensor::BinarySensor health;
  bool fault_notified{false};

 protected:
  void on_radio_failure_() override {
    CHECK(is_failed());
    CHECK(!health.state);
    fault_notified = true;
  }
};

static void stale_read_crosses_deadline(uint32_t start) {
  using namespace redlink_test;
  // Both old values latch early in calibration. Preemption during/after the
  // second SPI transfer delays its return until 3011 us; the next pair sees RX.
  reset(start, {{0x08, 16}, {0x08, 2995}, {0x0d}, {0x0d}});
  TestRadio radio;
  CHECK(radio.wait_state_(0x0d));
  CHECK(!radio.is_failed());
  CHECK(radio.health.state && !radio.fault_notified);
  CHECK(read_starts.size() == 4);
  CHECK(uint32_t(read_starts[1] - start) < 3000);
  CHECK(uint32_t(read_starts[2] - start) >= 3011);
}

static void stuck_rx_is_bounded(uint32_t start) {
  using namespace redlink_test;
  reset(start);
  TestRadio radio;
  CHECK(!radio.wait_state_(0x0d));
  CHECK(radio.is_failed());
  CHECK(!radio.health.state && radio.fault_notified);
  CHECK(read_starts.size() >= 2);
  // A failure must be based on a stable observation begun after the deadline,
  // not a pre-deadline sample returned late. No second 3 ms retry is allowed.
  CHECK(uint32_t(read_starts[read_starts.size() - 2] - start) >= 3000);
  CHECK(uint32_t(now - start) >= 3000 && uint32_t(now - start) <= 3100);
}

static void unstable_status() {
  using namespace redlink_test;
  // A lone RX sample is not success; four disagreeing reads must be retried.
  reset(0, {{0x08}, {0x0d}, {0x08}, {0x0d}, {0x08}, {0x0d}, {0x0d}});
  TestRadio recovered;
  CHECK(recovered.wait_state_(0x0d));
  CHECK(!recovered.is_failed());
  CHECK(read_starts.size() == 7);

  // Alternating values must neither accept RX nor spin forever.
  reset(0);
  for (size_t i = 0; i < 220; i++)
    samples.push_back({uint8_t(i % 2 == 0 ? 0x08 : 0x0d)});
  TestRadio unstable;
  CHECK(!unstable.wait_state_(0x0d));
  CHECK(unstable.is_failed());
  CHECK(read_starts.size() < samples.size());
  CHECK(read_starts.size() % 4 == 0);
  CHECK(read_starts[read_starts.size() - 4] >= 3000);
  CHECK(now >= 3000 && now <= 3200);
}

static void io_failure() {
  using namespace redlink_test;
  reset(0);
  TestRadio failed;
  failed.mark_failed();
  CHECK(!failed.wait_state_(0x0d));
  CHECK(read_starts.empty() && now == 0);

  // SO/MISO never becomes ready: propagate select_ failure without waiting
  // out the separate MARCSTATE deadline or attempting another status read.
  reset(0);
  TestRadio not_ready;
  not_ready.miso.high = true;
  CHECK(!not_ready.wait_state_(0x0d));
  CHECK(not_ready.is_failed());
  CHECK(!not_ready.health.state && not_ready.fault_notified);
  CHECK(read_starts.empty() && now == 2000);
}

static void learned_channel_registers_and_tx_guard() {
  using namespace redlink_test;
  using namespace esphome::redlink;
  reset(0);
  emulate_registers = true;
  TestRadio radio;
  radio.configure_bus();
  radio.setup();
  CHECK(!radio.is_failed());
  const auto check_tuning = [](uint8_t rank) {
    const HopChannel channel = candidate_channel(rank);
    const uint32_t frequency = 904195310 + (int32_t(channel.channel) - 10) * 199951;
    const uint32_t word = (uint64_t(frequency) * 65536 + 13000000) / 26000000;
    CHECK(registers[0x04] == (channel.sync >> 8) && registers[0x05] == (channel.sync & 0xff));
    CHECK(registers[0x0d] == (word >> 16) && registers[0x0e] == ((word >> 8) & 0xff) &&
          registers[0x0f] == (word & 0xff));
  };
  CHECK(radio.receive_candidate_ == 0);
  check_tuning(0);  // Initial RX is candidate channel4/sync636b, not hardcoded slot0/channel10.
  radio.enable_tx();
  CHECK(!radio.send_frame(status_query(0x389e, 0xb14f)) && tx_strobes == 0);
  for (uint8_t slot = 0; slot < HOP_COUNT; ++slot) {
    CHECK(radio.hop_.observe(slot, 49 - slot, 1000 + slot));
    CHECK(radio.hop_.observe(slot, 49 - slot, 2000 + slot));
  }
  CHECK(!radio.send_frame(status_query(0x389e, 0xb14f)));  // Wrong physical candidate.
  CHECK(radio.listen_candidate_(49));
  check_tuning(49);
  CHECK(radio.send_frame(status_query(0x389e, 0xb14f)) && tx_strobes == 1);
  emulate_registers = false;
}

static void delayed_loop_keeps_buffered_packet(uint32_t start) {
  using namespace redlink_test;
  using namespace esphome::redlink;
  reset(start);
  emulate_registers = true;
  TestRadio radio;
  radio.configure_bus();
  radio.setup();
  const Frame expected = status_query(0x389e, 0xb14f);
  rx_fifo.assign(expected.data.begin(), expected.data.begin() + expected.size);
  whiten(rx_fifo.data(), rx_fifo.size());
  registers[0x3b] = rx_fifo.size();
  // Scheduling was delayed, but the CC1101 continued receiving correctly.
  const uint32_t expected_sync = now - 5000 - 417;
  radio.delayed_sync(5000);
  Frame received;
  uint32_t sync;
  CHECK(radio.read_frame(received, sync));
  CHECK(received.size == expected.size);
  CHECK(std::equal(expected.data.begin(), expected.data.begin() + expected.size, received.data.begin()));
  CHECK(sync == expected_sync);
  // An empty FIFO really has stalled; reset it without accepting a frame.
  radio.delayed_sync(5000);
  CHECK(!radio.read_frame(received, sync));
  CHECK(!radio.is_failed());
  emulate_registers = false;
}

static esphome::redlink::Frame packet(size_t size) {
  using namespace esphome::redlink;
  Frame frame;
  CHECK(size >= 13 && size <= frame.data.size());
  frame.size = size;
  frame.data[0] = size - 3;
  for (size_t i = 1; i < size - 2; ++i)
    frame.data[i] = static_cast<uint8_t>(i * 37 + 11);
  const uint16_t crc = crc16(frame.data.data(), size - 2);
  frame.data[size - 2] = crc >> 8;
  frame.data[size - 1] = crc;
  CHECK(valid_frame(frame));
  return frame;
}

static esphome::redlink::Frame captured_status() {
  // Actual heat=71 status missed by the device in latency-final-rf-20261107T124417,
  // interval 11.391 s, channel 10. Keep its original payload and software CRC.
  constexpr char hex[] =
      "3428ffffc27a389e0000f01e81ff005a8046474a2e005cdc284e3c63464afc0280328446c8a901070020f00922c0004500a001000f0195";
  esphome::redlink::Frame frame;
  frame.size = (sizeof(hex) - 1) / 2;
  const auto digit = [](char c) { return c <= '9' ? c - '0' : c - 'a' + 10; };
  for (size_t i = 0; i < frame.size; ++i)
    frame.data[i] = (digit(hex[2 * i]) << 4) | digit(hex[2 * i + 1]);
  CHECK(esphome::redlink::valid_frame(frame));
  return frame;
}

static std::vector<uint8_t> on_air(const esphome::redlink::Frame &frame) {
  std::vector<uint8_t> bytes(frame.data.begin(), frame.data.begin() + frame.size);
  esphome::redlink::whiten(bytes.data(), bytes.size());
  return bytes;
}

static void same_packet(const esphome::redlink::Frame &actual, const esphome::redlink::Frame &expected) {
  CHECK(actual.size == expected.size);
  CHECK(std::equal(expected.data.begin(), expected.data.begin() + expected.size, actual.data.begin()));
}

static void setup_receiver(TestRadio &radio, uint32_t start = 10000) {
  using namespace redlink_test;
  reset(start);
  emulate_registers = true;
  radio.configure_bus();
  radio.setup();
  CHECK(!radio.is_failed());
}

static void rearmed(const TestRadio &radio) {
  using namespace redlink_test;
  CHECK(!radio.is_failed() && radio.health.state && !radio.fault_notified);
  CHECK(!radio.receiving() && !radio.gdo.high);
  CHECK(marcstate == 0x0d && rx_fifo.empty());
  CHECK(registers[0x08] == 0 && registers[0x06] == 64);
}

static void noisy_tail_preserves_short_packet(uint32_t start) {
  using namespace redlink_test;
  using namespace esphome::redlink;
  for (size_t size : {size_t(13), size_t(16), size_t(55), size_t(64)}) {
    TestRadio radio;
    setup_receiver(radio, start);
    const Frame expected = size == 55 ? captured_status() : packet(size);
    const auto wire = on_air(expected);
    radio.packet_sync();
    const uint32_t expected_sync = now - 417;
    receive_bytes(wire.data(), wire.size());
    receive_noise(128);
    advance(70000);  // Longer than both RX deadlines, with no main-loop service.
    CHECK(marcstate == 1 && !radio.gdo.high && rx_fifo.size() == 64);
    CHECK(registers[0x3b] == 64);  // Noise did not cause overflow.
    Frame received;
    uint32_t sync = 0;
    CHECK(radio.read_frame(received, sync));
    same_packet(received, expected);
    CHECK(sync == expected_sync);
    rearmed(radio);
  }
}

static void streamed_packets_stop_at_declared_length(uint32_t start) {
  using namespace redlink_test;
  using namespace esphome::redlink;
  for (size_t size : {size_t(65), size_t(75), size_t(255), size_t(256), size_t(257), size_t(258)}) {
    TestRadio radio;
    setup_receiver(radio, start);
    const Frame expected = packet(size);
    const auto wire = on_air(expected);
    radio.packet_sync();
    const uint32_t expected_sync = now - 417;
    Frame received;
    uint32_t sync = 0;
    receive_bytes(wire.data(), 2);
    CHECK(!radio.read_frame(received, sync));
    CHECK(rx_fifo.size() == 1 && radio.gdo.high);
    CHECK(registers[0x06] == static_cast<uint8_t>(size));
    CHECK(registers[0x08] == (size > 255 ? 2 : 0));
    size_t offset = 2;
    while (size - offset > 32) {
      receive_bytes(wire.data() + offset, 32);
      offset += 32;
      CHECK(!radio.read_frame(received, sync));
      CHECK(rx_fifo.size() == 1 && radio.gdo.high && registers[0x08] == 0);
    }
    receive_bytes(wire.data() + offset, size - offset);
    receive_noise(128);
    advance(70000);
    CHECK(marcstate == 1 && !radio.gdo.high && rx_wire_count == size);
    CHECK(radio.read_frame(received, sync));
    same_packet(received, expected);
    CHECK(sync == expected_sync);
    rearmed(radio);
  }
}

static void reject_corrupt_and_overflowed_packets() {
  using namespace redlink_test;
  using namespace esphome::redlink;
  for (unsigned fault = 0; fault < 4; ++fault) {
    TestRadio radio;
    setup_receiver(radio);
    const Frame expected = captured_status(), unchanged = packet(13);
    auto wire = on_air(expected);
    if (fault == 0)
      wire[20] ^= 1;  // Payload corruption, original CRC.
    if (fault == 1)
      wire[0] = 0xff;  // Decoded length zero.
    radio.packet_sync();
    receive_bytes(wire.data(), wire.size());
    if (fault == 2)
      registers[0x3b] = 0x80 | wire.size();  // Never salvage a CRC-valid overflow prefix.
    if (fault == 3)
      registers[0x3b] = 65;  // Impossible/unsafe FIFO count, even without its overflow bit.
    Frame received = unchanged;
    uint32_t sync = 0x12345678;
    CHECK(!radio.read_frame(received, sync));
    same_packet(received, unchanged);
    CHECK(sync == 0x12345678);
    rearmed(radio);
    // Error recovery must receive the next packet, not just clear software flags.
    wire = on_air(expected);
    radio.packet_sync();
    receive_bytes(wire.data(), wire.size());
    CHECK(radio.read_frame(received, sync));
    same_packet(received, expected);
    rearmed(radio);
  }
}

static void long_packet_requires_fifo_service() {
  using namespace redlink_test;
  using namespace esphome::redlink;
  for (unsigned fault = 0; fault < 3; ++fault) {
    TestRadio radio;
    setup_receiver(radio);
    const auto wire = on_air(packet(75));
    Frame received = packet(13), unchanged = received;
    uint32_t sync = 0x12345678;
    radio.packet_sync();
    if (fault == 0) {
      receive_bytes(wire.data(), wire.size());  // Initial cap reached before length was decoded.
      CHECK(marcstate == 1 && rx_fifo.size() == 64);
    } else {
      receive_bytes(wire.data(), 2);
      if (fault == 1) {
        CHECK(!radio.read_frame(received, sync));  // Extend length, then starve a genuinely long frame.
        receive_bytes(wire.data() + 2, wire.size() - 2);
        CHECK(marcstate == 0x11 && (registers[0x3b] & 0x80) != 0);
      } else {
        // Preemption at the PKTLEN write: hardware already stopped at its old cap.
        before_register_write = [&wire](uint8_t address) {
          CHECK(address == 0x06);
          receive_bytes(wire.data() + 2, wire.size() - 2);
        };
      }
    }
    CHECK(!radio.read_frame(received, sync));
    if (fault == 2)
      CHECK(!radio.read_frame(received, sync));  // Reobserve FIFO after its concurrent hardware end.
    same_packet(received, unchanged);
    CHECK(sync == 0x12345678);
    rearmed(radio);
  }
}

static void incomplete_packet_deadlines(uint32_t start) {
  using namespace redlink_test;
  using namespace esphome::redlink;
  for (unsigned fault = 0; fault < 4; ++fault) {
    TestRadio radio;
    setup_receiver(radio, start);
    Frame received = packet(13), unchanged = received;
    uint32_t sync = 0x12345678;
    const auto wire = on_air(packet(75));
    radio.packet_sync();
    if (fault == 0) {
      advance(3000);  // Sync with no FIFO progress.
    } else {
      receive_bytes(wire.data(), 2);
      CHECK(!radio.read_frame(received, sync));
      if (fault == 1) {
        advance(2999);
        CHECK(!radio.read_frame(received, sync) && radio.receiving());
        advance(1);  // One held-back byte is not new progress.
      } else {
        advance(65000);
        if (fault == 2)
          receive_bytes(wire.data() + 2, 32);  // Some progress cannot extend the total packet deadline.
        else
          rx_status_samples = {1, 2, 3, 4};  // An unstable status cannot postpone expiry indefinitely.
      }
    }
    CHECK(!radio.read_frame(received, sync));
    same_packet(received, unchanged);
    CHECK(sync == 0x12345678);
    rearmed(radio);
  }
}

static void fifo_observation_and_sync_races(uint32_t start) {
  using namespace redlink_test;
  using namespace esphome::redlink;
  for (unsigned scenario = 0; scenario < 5; ++scenario) {
    TestRadio radio;
    setup_receiver(radio, start);
    const Frame expected = captured_status();
    const auto wire = on_air(expected);
    Frame received;
    uint32_t sync = 0;
    const uint32_t expected_sync = now - 417;
    if (scenario == 0) {
      after_rx_status = [&radio, &wire]() {
        radio.packet_sync();  // IRQ arrives during an initially inactive FIFO observation.
        receive_bytes(wire.data(), wire.size());
      };
      CHECK(!radio.read_frame(received, sync));
      CHECK(rx_strobes == 1 && radio.receiving());
    } else {
      radio.packet_sync();
      if (scenario == 1) {
        receive_bytes(wire.data(), wire.size());
        rx_status_samples = {55, 54, 55, 54};
        CHECK(!radio.read_frame(received, sync));
        CHECK(rx_fifo.size() == wire.size() && rx_strobes == 1);
      } else if (scenario == 2 || scenario == 4) {
        receive_bytes(wire.data(), 2);
        rx_status_samples = {2, 2};
        after_rx_status = [&wire, scenario]() {
          receive_bytes(wire.data() + 2, wire.size() - 2);
          if (scenario == 4)
            receive_noise(100);  // GDO falls after the old FIFO count was latched.
          advance(70000);        // Old FIFO observation returns after the packet deadline.
        };
        CHECK(!radio.read_frame(received, sync));
        CHECK(rx_strobes == 1 && radio.receiving());
      } else {
        receive_bytes(wire.data(), wire.size());
        before_idle = []() { receive_noise(100); };  // Noise arrives before manual SIDLE.
      }
    }
    CHECK(radio.read_frame(received, sync));
    same_packet(received, expected);
    CHECK(sync == expected_sync);
    rearmed(radio);
  }
  TestRadio missing_irq;
  setup_receiver(missing_irq, start);
  const auto wire = on_air(captured_status());
  missing_irq.gdo.high = true;
  receive_bytes(wire.data(), wire.size());
  Frame received;
  uint32_t sync = 0;
  CHECK(!missing_irq.read_frame(received, sync));  // No invented sync timestamp.
  rearmed(missing_irq);
}

int main() {
  stale_read_crosses_deadline(0);
  stale_read_crosses_deadline(UINT32_MAX - 1500);
  stuck_rx_is_bounded(0);
  stuck_rx_is_bounded(UINT32_MAX - 1500);
  unstable_status();
  io_failure();
  learned_channel_registers_and_tx_guard();
  delayed_loop_keeps_buffered_packet(10000);
  delayed_loop_keeps_buffered_packet(UINT32_MAX - 1000);
  noisy_tail_preserves_short_packet(10000);
  noisy_tail_preserves_short_packet(UINT32_MAX - 1000);
  streamed_packets_stop_at_declared_length(10000);
  streamed_packets_stop_at_declared_length(UINT32_MAX - 1000);
  reject_corrupt_and_overflowed_packets();
  long_packet_requires_fifo_service();
  incomplete_packet_deadlines(10000);
  incomplete_packet_deadlines(UINT32_MAX - 1000);
  fifo_observation_and_sync_races(10000);
  fifo_observation_and_sync_races(UINT32_MAX - 1000);
  std::puts("RedLINK radio receive and wait-state tests passed");
}
