// Compile the actual driver; fake only ESPHome's clock, GPIO, SPI and component surface.
#include "radio.h"
#include "esphome/core/hal.h"

#include "check.h"
#include <algorithm>
#include <cstdio>
#include <initializer_list>
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
static uint8_t registers[64]{};
static uint8_t marcstate = 1;
static unsigned tx_strobes = 0;
static std::vector<uint8_t> rx_fifo;

static void advance(uint32_t duration) {
  now += duration;
  // Fail instead of hanging if production loses its bounded timeout.
  CHECK(uint32_t(now - started) <= 10000);
}
static void reset(uint32_t start, std::initializer_list<Sample> script = {}, Sample rest = {0x08}) {
  now = started = start;
  samples = script;
  read_starts.clear();
  fallback = rest;
}
void transfer_status(uint8_t *data, size_t size) {
  if (emulate_registers) {
    CHECK(size == 2);
    const uint8_t address = data[0] & 0x3f;
    data[1] = address == 0x35 ? marcstate : address == 0x30 ? 0 : address == 0x31 ? 0x14 : registers[address];
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
  if (command == 0x36)
    marcstate = 1;
  if (command == 0x34)
    marcstate = 0x0d;
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
  registers[data[0]] = data[1];
}
void read_bytes(uint8_t *data, size_t size) {
  CHECK(emulate_registers && size <= rx_fifo.size());
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
  void delayed_sync(uint32_t age) {
    sync_end_us_ = esphome::micros() - age;
    sync_pending_ = true;
    gdo.high = true;
  }
  void configure_bus() {
    parent_ = this;
    cs_ = &cs;
    set_gdo0_pin(&gdo);
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
  std::puts("RedLINK radio wait-state tests passed");
}
