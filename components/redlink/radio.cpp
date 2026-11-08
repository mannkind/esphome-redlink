#include "radio.h"

#include <algorithm>
#include <cstring>

#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome::redlink {
namespace {

const char *const TAG = "redlink.radio";

constexpr uint8_t IOCFG0 = 0x02;
constexpr uint8_t SYNC1 = 0x04;
constexpr uint8_t SYNC0 = 0x05;
constexpr uint8_t PKTLEN = 0x06;
constexpr uint8_t PKTCTRL0 = 0x08;
constexpr uint8_t FREQ2 = 0x0d;
constexpr uint8_t FSCAL1 = 0x25;
constexpr uint8_t PARTNUM = 0x30;
constexpr uint8_t VERSION = 0x31;
constexpr uint8_t MARCSTATE = 0x35;
constexpr uint8_t TXBYTES = 0x3a;
constexpr uint8_t RXBYTES = 0x3b;
constexpr uint8_t PATABLE = 0x3e;
constexpr uint8_t FIFO = 0x3f;
constexpr uint8_t SRES = 0x30;
constexpr uint8_t SRX = 0x34;
constexpr uint8_t STX = 0x35;
constexpr uint8_t SIDLE = 0x36;
constexpr uint8_t SFRX = 0x3a;
constexpr uint8_t SFTX = 0x3b;
constexpr uint8_t STATE_IDLE = 0x01;
constexpr uint8_t STATE_RX = 0x0d;
constexpr uint8_t STATE_TX_UNDERFLOW = 0x16;
constexpr uint32_t READY_TIMEOUT_US = 2000;
constexpr uint32_t STATE_TIMEOUT_US = 3000;
// A maximum-length frame takes about 54 ms; extended clock broadcasts exceed one FIFO.
constexpr uint32_t RX_PACKET_TIMEOUT_US = 65000;
constexpr uint32_t RX_STALL_TIMEOUT_US = 3000;
// DRATE_E=10, DRATE_M=0x83, 26 MHz: 38383.4838867 symbols/second.
// GDO0=0x06 rises at sync END; report the first of its 16 sync bits.
constexpr uint32_t SYNC_DURATION_US = 417;
constexpr uint32_t APPROX_BIT_RATE = 38383;

struct RegisterSetting {
  uint8_t address;
  uint8_t value;
};

// 915 MHz / 38.4 kbaud sensitivity profile, with the measured RedLINK packet format.
// Hardware whitening and CRC must remain OFF: the supplied CRC and length are whitened too.
// TEST0=0x09 / FSCAL2=0x2a select the high VCO for the supported 902-928 MHz band.
constexpr RegisterSetting SETTINGS[] = {
    {0x00, 0x2e},                    // GDO2 high impedance.
    {0x01, 0x2e},                    // GDO1/SO high impedance whenever CS is high.
    {IOCFG0, 0x06},   {0x03, 0x47},  // ADC retention for narrow RX filter; FIFO threshold 32 bytes.
    {PKTLEN, 64},     {0x07, 0x00},  // No appended status, autoflush, or address filtering.
    {PKTCTRL0, 0x00},                // Fixed-length RX cap; software decodes the whitened length.
    {0x09, 0x00},     {0x0a, 0x00},  // CHANNR=0: frequency_ is the actual channel-10 carrier, not a hop base.
    {0x0b, 0x06},                    // 152.344 kHz IF.
    {0x0c, 0x00},     {0x10, 0xca},  // RX bandwidth 101.5625 kHz; data-rate exponent 10.
    {0x11, 0x83},     {0x12, 0x12},  // GFSK (BT=0.5), no Manchester, exact 16/16-bit sync.
    {0x13, 0x62},                    // No FEC; 16 preamble bytes.
    {0x14, 0xf8},     {0x15, 0x34},  // +/-19042.96875 Hz deviation.
    {0x16, 0x07},                    // No RX termination timer.
    {0x17, 0x00},                    // No CCA; TX and bounded RX end in IDLE.
    {0x18, 0x18},                    // Calibrate on every IDLE -> RX/TX, regulator timeout 64 cycles.
    {0x19, 0x16},     {0x1a, 0x6c}, {0x1b, 0x43},   {0x1c, 0x40},
    {0x1d, 0x91},     {0x21, 0x56}, {0x22, 0x10},  // PATABLE index zero, no PA ramp table.
    {0x23, 0xe9},     {0x24, 0x2a}, {FSCAL1, 0x00}, {0x26, 0x1f},
    {0x2c, 0x81},     {0x2d, 0x35}, {0x2e, 0x09},
};

}  // namespace

void IRAM_ATTR RedLinkRadio::sync_interrupt_(RedLinkRadio *radio) {
  if (!radio->sync_armed_)
    return;
  radio->sync_end_us_ = micros();
  radio->sync_armed_ = false;
  radio->sync_pending_ = true;
}

void RedLinkRadio::disarm_sync_() {
  InterruptLock lock;
  this->sync_armed_ = false;
  this->sync_pending_ = false;
}

void RedLinkRadio::fail_radio_(const char *reason) {
  this->disarm_sync_();
  this->rx_running_ = false;
  this->rx_active_ = false;
  this->tx_active_ = false;
  this->initialized_ = false;
  ESP_LOGE(TAG, "%s; radio disabled", reason);
  this->mark_failed();
  if (radio_healthy_ != nullptr)
    radio_healthy_->publish_state(false);
  on_radio_failure_();
}

bool RedLinkRadio::wait_ready_() {
  const uint32_t started = micros();
  while (this->miso_pin_->digital_read()) {
    if (micros() - started >= READY_TIMEOUT_US)
      return false;
    delayMicroseconds(1);
  }
  return true;
}

bool RedLinkRadio::select_() {
  if (this->is_failed())
    return false;
  this->enable();
  if (this->wait_ready_())
    return true;
  this->disable();
  this->fail_radio_("CC1101 SO/MISO ready timeout");
  return false;
}

bool RedLinkRadio::strobe_(uint8_t command) {
  if ((receive_only_ || !transmit_enabled_) && command == STX)
    return false;
  if (!this->select_())
    return false;
  this->transfer_byte(command);
  this->disable();
  return true;
}

bool RedLinkRadio::read_register_(uint8_t address, uint8_t &value) {
  if (!this->select_())
    return false;
  // Status addresses need the burst bit even though exactly ONE status register is read.
  uint8_t bytes[2] = {static_cast<uint8_t>(address | (address >= 0x30 ? 0xc0 : 0x80)), 0};
  this->transfer_array(bytes, sizeof(bytes));
  this->disable();
  value = bytes[1];
  return true;
}

bool RedLinkRadio::read_stable_status_(uint8_t address, uint8_t &value) {
  // TI SWRZ020E: continuously updated status fields require two identical consecutive reads.
  // Bound each batch to four reads; transient disagreement is not a valid state sample.
  uint8_t previous;
  if (!this->read_register_(address, previous))
    return false;
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    uint8_t current;
    if (!this->read_register_(address, current))
      return false;
    if (current == previous) {
      value = current;
      return true;
    }
    previous = current;
  }
  return false;
}

bool RedLinkRadio::write_register_(uint8_t address, uint8_t value) {
  if (!this->select_())
    return false;
  const uint8_t bytes[2] = {address, value};
  this->write_array(bytes, sizeof(bytes));
  this->disable();
  return true;
}

bool RedLinkRadio::read_fifo_(uint8_t *data, size_t size) {
  if (size == 0 || size > 64 || !this->select_())
    return false;
  // A one-byte read is SINGLE access, not a one-byte FIFO burst.
  this->transfer_byte(static_cast<uint8_t>(FIFO | (size == 1 ? 0x80 : 0xc0)));
  this->read_array(data, size);
  this->disable();
  return true;
}

bool RedLinkRadio::write_fifo_(const uint8_t *data, size_t size) {
  if (size == 0 || size > 64 || !this->select_())
    return false;
  this->transfer_byte(static_cast<uint8_t>(FIFO | (size == 1 ? 0 : 0x40)));
  this->write_array(data, size);
  this->disable();
  return true;
}

bool RedLinkRadio::wait_state_(uint8_t state) {
  const uint32_t started = micros();
  do {
    // SPI/RTOS preemption can return a pre-deadline sample after the deadline.
    // Only fail on an observation STARTED after expiry, not on a stale sample.
    const bool expired = micros() - started >= STATE_TIMEOUT_US;
    uint8_t current = 0xff;
    if (this->read_stable_status_(MARCSTATE, current) && (current & 0x1f) == state) {
      if (expired)
        ESP_LOGW(TAG, "State 0x%02X confirmed by post-deadline observation after %u us", state,
                 unsigned(micros() - started));
      return true;
    }
    if (this->is_failed())
      return false;
    // Keep the same deadline; this allows one final bounded status observation,
    // not a restarted timeout, radio reset, or acceptance of an intermediate state.
    if (expired) {
      ESP_LOGE(TAG, "State transition timed out: wanted 0x%02X, observed 0x%02X after %u us", state, current,
               unsigned(micros() - started));
      break;
    }
    delayMicroseconds(5);
  } while (true);
  this->fail_radio_("CC1101 MARCSTATE timeout");
  return false;
}

bool RedLinkRadio::enter_idle_() {
  this->disarm_sync_();
  this->tx_prepared_ = false;
  this->rx_running_ = false;
  return this->strobe_(SIDLE) && this->wait_state_(STATE_IDLE);
}

void RedLinkRadio::setup() {
  if (this->parent_ == nullptr || this->cs_ == nullptr || this->gdo0_pin_ == nullptr || this->miso_pin_ == nullptr ||
      !this->cs_->is_internal() || static_cast<InternalGPIOPin *>(this->cs_)->is_inverted() ||
      this->gdo0_pin_->is_inverted() || this->miso_pin_->is_inverted() || this->frequency_ < 903199706 ||
      this->frequency_ > 905805439) {
    this->fail_radio_("Invalid RedLINK SPI pins or tuning reference (all candidates must fit 902-928 MHz)");
    return;
  }
  this->gdo0_pin_->setup();
  this->gdo0_pin_->pin_mode(gpio::FLAG_INPUT);
  // The SPI bus owns MISO's input/matrix configuration; only sample that pin here.
  this->spi_setup();
  if (!this->spi_is_ready()) {
    this->fail_radio_("SPI bus is not ready");
    return;
  }

  // Assumes a supply meeting the CC1101 automatic power-on-reset requirements.
  // Pulse CS, allow the oscillator to settle, then SRES with SO-ready checks on both sides.
  this->enable();
  delayMicroseconds(1);
  this->disable();
  delayMicroseconds(41);
  if (!this->select_())
    return;
  this->transfer_byte(SRES);
  delayMicroseconds(1);
  const bool reset_ready = this->wait_ready_();
  this->disable();
  if (!reset_ready) {
    this->fail_radio_("CC1101 reset timeout");
    return;
  }

  uint8_t part;
  if (!this->read_register_(PARTNUM, part) || !this->read_register_(VERSION, this->chip_version_))
    return;
  if (part != 0x00 || this->chip_version_ == 0x00 || this->chip_version_ == 0xff) {
    this->fail_radio_("CC1101 chip identification failed");
    return;
  }

  for (const auto &setting : SETTINGS) {
    uint8_t actual;
    if (!this->write_register_(setting.address, setting.value) || !this->read_register_(setting.address, actual))
      return;
    if (actual != setting.value) {
      this->fail_radio_("CC1101 configuration readback failed");
      return;
    }
  }
  const auto initial = candidate_channel(this->receive_candidate_);
  const uint32_t initial_frequency = this->frequency_ + (int32_t(initial.channel) - 10) * 199951;
  const uint32_t frequency_word = (uint64_t(initial_frequency) * 65536ULL + 13000000ULL) / 26000000ULL;
  const RegisterSetting channel[] = {
      {SYNC1, static_cast<uint8_t>(initial.sync >> 8)},
      {SYNC0, static_cast<uint8_t>(initial.sync)},
      {FREQ2, static_cast<uint8_t>(frequency_word >> 16)},
      {static_cast<uint8_t>(FREQ2 + 1), static_cast<uint8_t>(frequency_word >> 8)},
      {static_cast<uint8_t>(FREQ2 + 2), static_cast<uint8_t>(frequency_word)},
      {PATABLE, 0x8e},  // Approximately 0 dBm on the TI 915 MHz reference design.
  };
  for (const auto &setting : channel) {
    uint8_t actual;
    if (!this->write_register_(setting.address, setting.value) || !this->read_register_(setting.address, actual))
      return;
    if (actual != setting.value) {
      this->fail_radio_("CC1101 channel/PA readback failed");
      return;
    }
  }

  this->initialized_ = true;
  this->gdo0_pin_->attach_interrupt(&RedLinkRadio::sync_interrupt_, this, gpio::INTERRUPT_RISING_EDGE);
  this->begin_rx();
  if (radio_healthy_ != nullptr)
    radio_healthy_->publish_state(initialized_ && !is_failed());
}

void RedLinkRadio::dump_config() {
  ESP_LOGCONFIG(TAG, "RedLINK CC1101 (26 MHz crystal):");
  LOG_SPI_DEVICE(this);
  LOG_PIN("  GDO0 sync interrupt: ", this->gdo0_pin_);
  LOG_PIN("  SO/MISO ready input: ", this->miso_pin_);
  ESP_LOGCONFIG(TAG, "  Chip version: 0x%02X; channel-10 tuning reference: %u Hz", this->chip_version_,
                static_cast<unsigned>(this->frequency_));
  ESP_LOGCONFIG(TAG, "  GFSK BT=0.5; 38383.484 baud; deviation 19042.969 Hz; RX BW 101562.5 Hz");
  ESP_LOGCONFIG(TAG, "  16-byte preamble; software PN9/CRC; nominal 0 dBm; learned logical slot-0 TX only");
  ESP_LOGCONFIG(TAG, "  Sync timestamp: GPIO ISR minus 417 us; IDLE->RX/TX autocalibration enabled");
  ESP_LOGCONFIG(TAG, "  Receive-only enforcement: %s", receive_only_ ? "ON (TX blocked)" : "OFF");
}

bool RedLinkRadio::listen_candidate_(uint8_t rank) {
  if (rank >= HOP_COUNT || this->tx_active_ || this->is_failed())
    return false;
  if (rank == this->receive_candidate_)
    return true;
  if (!this->enter_idle_())
    return false;
  const auto channel = candidate_channel(rank);
  const uint32_t frequency = this->frequency_ + (int32_t(channel.channel) - 10) * 199951;
  const uint32_t word = (uint64_t(frequency) * 65536ULL + 13000000ULL) / 26000000ULL;
  const uint16_t sync = channel.sync;
  if (!this->write_register_(SYNC1, sync >> 8) || !this->write_register_(SYNC0, sync) ||
      !this->write_register_(FREQ2, word >> 16) || !this->write_register_(FREQ2 + 1, word >> 8) ||
      !this->write_register_(FREQ2 + 2, word))
    return false;
  this->receive_candidate_ = rank;
  this->begin_rx();
  return !this->is_failed();
}

void RedLinkRadio::begin_rx() {
  if (!this->initialized_ || this->is_failed() || this->tx_active_)
    return;
  if (!this->enter_idle_() || !this->strobe_(SFRX) || !this->write_register_(PKTCTRL0, 0x00) ||
      !this->write_register_(PKTLEN, 64))
    return;
  this->rx_frame_.size = 0;
  this->rx_expected_ = 0;
  this->rx_active_ = false;
  this->rx_infinite_ = false;
  {
    InterruptLock lock;
    this->sync_pending_ = false;
    this->sync_armed_ = true;
  }
  this->rx_running_ = true;
  if (!this->strobe_(SRX) || !this->wait_state_(STATE_RX))
    return;
  uint8_t calibration;
  if (!this->read_register_(FSCAL1, calibration))
    return;
  if ((calibration & 0x3f) == 0x3f) {
    this->enter_idle_();
    this->fail_radio_("CC1101 PLL calibration failed");
  }
}

bool RedLinkRadio::read_frame(Frame &frame, uint32_t &sync_start_us) {
  if (!this->initialized_ || this->is_failed() || this->tx_active_ || !this->rx_running_)
    return false;
  {
    InterruptLock lock;
    if (this->sync_pending_) {
      this->rx_sync_end_us_ = this->sync_end_us_;
      this->rx_last_progress_us_ = this->sync_end_us_;
      this->sync_pending_ = false;
      this->rx_active_ = true;
    }
  }
  // Observe buffered completion before applying a wall-clock deadline. A late
  // caller must not discard a packet the hardware already received and stopped.
  const bool expired = this->rx_active_ && micros() - this->rx_sync_end_us_ >= RX_PACKET_TIMEOUT_US;
  const bool stopped = this->rx_active_ && !this->gdo0_pin_->digital_read();
  uint8_t bytes;
  if (!this->read_stable_status_(RXBYTES, bytes)) {
    if (expired && !this->is_failed())
      this->begin_rx();
    return false;
  }
  const size_t available = bytes & 0x7f;
  if ((bytes & 0x80) != 0 || available > 64) {
    this->begin_rx();
    return false;
  }
  if (!this->rx_active_) {
    // FIFO content without an ISR timestamp must never be used to synchronize the protocol clock.
    // Recheck the handoff in case sync arrived during the SPI status read.
    bool pending;
    {
      InterruptLock lock;
      pending = this->sync_pending_;
    }
    if (!pending && (available != 0 || this->gdo0_pin_->digital_read()))
      this->begin_rx();
    return false;
  }
  // A GDO falling edge after the status observation can make its FIFO count
  // stale. Hardware is now stopped: keep the packet for a fresh next-loop read.
  if (!stopped && !this->gdo0_pin_->digital_read())
    return false;

  size_t unread = available;
  if (this->rx_expected_ == 0) {
    // The first whitened byte is not a hardware length byte. Start with a
    // 64-byte fixed cap so post-frame noise cannot overflow a short packet
    // while the main loop is delayed. Never empty an active FIFO (SWRZ020E).
    if (unread < 2) {
      if (expired || stopped || micros() - this->rx_last_progress_us_ >= RX_STALL_TIMEOUT_US)
        this->begin_rx();
      return false;
    }
    if (!this->read_fifo_(this->rx_frame_.data.data(), 1))
      return false;
    this->rx_frame_.size = 1;
    unread--;
    uint8_t length = this->rx_frame_.data[0];
    whiten(&length, 1);
    if (length == 0 || length > this->rx_frame_.data.size() - 3) {
      this->begin_rx();
      return false;
    }
    this->rx_expected_ = static_cast<size_t>(length) + 3;
    this->rx_last_progress_us_ = micros();
    if (this->rx_expected_ > 64) {
      // TI SWRS061I 15.2.1: PKTLEN can be extended during RX. If the first
      // FIFO has already filled and stopped, this longer packet is truncated.
      if (expired || !this->gdo0_pin_->digital_read()) {
        this->begin_rx();
        return false;
      }
      // 256..258 bytes need the modulo-256 counter (15.2.2). Keep infinite
      // mode only until we have read past the first occurrence of that count.
      if (this->rx_expected_ > 255) {
        if (!this->write_register_(PKTCTRL0, 0x02))
          return false;
        this->rx_infinite_ = true;
      }
      if (!this->write_register_(PKTLEN, static_cast<uint8_t>(this->rx_expected_)))
        return false;
    }
  }

  const size_t remaining = this->rx_expected_ - this->rx_frame_.size;
  if (unread >= remaining) {
    // Stop before draining the last byte, including when our short-packet
    // safety cap has not yet been reached. Never accept trailing noise.
    if (!this->enter_idle_())
      return false;
    uint8_t stopped_bytes;
    if (!this->read_stable_status_(RXBYTES, stopped_bytes)) {
      this->begin_rx();
      return false;
    }
    if ((stopped_bytes & 0x80) != 0 || (stopped_bytes & 0x7f) < remaining || (stopped_bytes & 0x7f) > 64) {
      this->begin_rx();
      return false;
    }
    if (!this->read_fifo_(this->rx_frame_.data.data() + this->rx_frame_.size, remaining))
      return false;
    this->rx_frame_.size = this->rx_expected_;
    whiten(this->rx_frame_.data.data(), this->rx_frame_.size);
    const bool valid = valid_frame(this->rx_frame_);
    if (valid) {
      frame.size = this->rx_frame_.size;
      std::memcpy(frame.data.data(), this->rx_frame_.data.data(), frame.size);
      sync_start_us = this->rx_sync_end_us_ - SYNC_DURATION_US;
    }
    if (!valid)
      this->begin_rx();
    return valid;
  }
  // Low GDO now denotes a hardware packet end, not necessarily an error.
  // It is an error only when the declared frame is still incomplete.
  if (!stopped && !this->gdo0_pin_->digital_read())
    return false;
  if (expired || stopped || (unread <= 1 && micros() - this->rx_last_progress_us_ >= RX_STALL_TIMEOUT_US)) {
    this->begin_rx();
    return false;
  }
  if (unread <= 1)
    return false;
  const size_t count = std::min(unread - 1, remaining);
  if (!this->read_fifo_(this->rx_frame_.data.data() + this->rx_frame_.size, count))
    return false;
  this->rx_frame_.size += count;
  this->rx_last_progress_us_ = micros();
  if (this->rx_infinite_ && this->rx_frame_.size > (this->rx_expected_ & 0xff)) {
    if (!this->write_register_(PKTCTRL0, 0x00))
      return false;
    this->rx_infinite_ = false;
  }
  return false;
}

bool RedLinkRadio::prepare_frame(const Frame &frame) {
  this->tx_succeeded_ = false;
  this->tx_prepared_ = false;
  if (receive_only_ || !transmit_enabled_ || !this->initialized_ || this->is_failed() || this->tx_active_ ||
      !hop_.ready() || receive_candidate_ != hop_.active_rank(0) || frame.size < 4 || frame.size > 64 ||
      !valid_frame(frame))
    return false;
  if (!this->enter_idle_() || !this->strobe_(SFRX) || !this->strobe_(SFTX) || !this->write_register_(PKTCTRL0, 0x00) ||
      !this->write_register_(PKTLEN, static_cast<uint8_t>(frame.size)))
    return false;
  this->rx_active_ = false;
  std::array<uint8_t, 64> whitened;
  std::memcpy(whitened.data(), frame.data.data(), frame.size);
  whiten(whitened.data(), frame.size);
  if (!this->write_fifo_(whitened.data(), frame.size))
    return false;
  uint8_t queued;
  if (!this->read_stable_status_(TXBYTES, queued) || queued != frame.size) {
    if (!this->is_failed())
      this->begin_rx();
    return false;
  }

  // All data are queued before STX. Calibration/settling takes ~0.8 ms, before the preamble.
  // The deadline includes 16 preamble bytes + 2 sync bytes + the complete whitened frame.
  this->tx_timeout_us_ =
      ((static_cast<uint32_t>(frame.size) + 18) * 8000000UL + APPROX_BIT_RATE - 1) / APPROX_BIT_RATE + 5000;
  this->tx_prepared_ = true;
  return true;
}

bool RedLinkRadio::start_tx() {
  const bool prepared = this->tx_prepared_;
  this->tx_prepared_ = false;
  if (!prepared || receive_only_ || !transmit_enabled_ || !this->initialized_ || this->is_failed() ||
      this->tx_active_ || !hop_.ready() || receive_candidate_ != hop_.active_rank(0))
    return false;
  {
    InterruptLock lock;
    this->sync_pending_ = false;
    this->sync_armed_ = true;
  }
  this->tx_started_us_ = micros();
  this->tx_active_ = true;
  return this->strobe_(STX);
}

bool RedLinkRadio::tx_finished() {
  if (!this->tx_active_ || this->is_failed())
    return true;
  uint8_t state;
  if (!this->read_stable_status_(MARCSTATE, state))
    return this->is_failed();
  state &= 0x1f;
  if (state == STATE_TX_UNDERFLOW) {
    ESP_LOGW(TAG, "TX FIFO underflow; packet dropped");
    this->tx_active_ = false;
    if (this->enter_idle_())
      this->strobe_(SFTX);
    return true;
  }
  if (state == STATE_IDLE) {
    // MARCSTATE can briefly say IDLE before STX has begun: queued bytes distinguish that case.
    uint8_t queued;
    if (!this->read_stable_status_(TXBYTES, queued))
      return this->is_failed();
    if (queued == 0) {
      this->tx_active_ = false;
      this->tx_succeeded_ = true;
      return true;
    }
  }
  // The loop can run late. Observe completed TX before enforcing its deadline;
  // elapsed wall time alone does not mean the radio is still transmitting.
  if (micros() - this->tx_started_us_ >= this->tx_timeout_us_) {
    this->enter_idle_();
    // Do not risk a further transmission after an unexplained abort (TX-byte repetition erratum).
    this->fail_radio_("CC1101 transmission timeout");
    return true;
  }
  return false;
}

}  // namespace esphome::redlink
