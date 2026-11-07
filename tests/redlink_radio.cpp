// Compile the actual driver; fake only ESPHome's clock, GPIO, SPI and component surface.
#include "radio.h"
#include "esphome/core/hal.h"

#include "check.h"
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
  CHECK(size == 2 && data[0] == 0xf5 && data[1] == 0);  // Burst-read MARCSTATE only.
  const size_t index = read_starts.size();
  read_starts.push_back(now);
  const Sample sample = index < samples.size() ? samples[index] : fallback;
  advance(sample.duration_us);
  data[1] = sample.value;
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

int main() {
  stale_read_crosses_deadline(0);
  stale_read_crosses_deadline(UINT32_MAX - 1500);
  stuck_rx_is_bounded(0);
  stuck_rx_is_bounded(UINT32_MAX - 1500);
  unstable_status();
  io_failure();
  std::puts("RedLINK radio wait-state tests passed");
}
