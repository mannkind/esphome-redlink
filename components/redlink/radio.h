#pragma once

#include "protocol.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"

namespace esphome::redlink {

class RedLinkRadio : public Component,
                     public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW, spi::CLOCK_PHASE_LEADING,
                                           spi::DATA_RATE_1MHZ> {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  void set_gdo0_pin(InternalGPIOPin *pin) { this->gdo0_pin_ = pin; }
  // The same physical SO/MISO pin configured on the SPI bus, not a second radio connection.
  void set_miso_pin(InternalGPIOPin *pin) { this->miso_pin_ = pin; }
  void set_frequency(uint32_t frequency) { this->frequency_ = frequency; }
  void set_sync_word(uint16_t sync_word) { this->sync_word_ = sync_word; }
  void set_receive_only(bool value) { receive_only_ = value; }
  void set_radio_healthy_sensor(binary_sensor::BinarySensor *value) { radio_healthy_ = value; }

  // Call frequently from the owning component's loop. RX is rearmed after every packet/error.
  bool read_frame(Frame &frame, uint32_t &sync_start_us);
  // Starts fixed-length TX; never waits for the packet to finish. Call begin_rx() after tx_finished().
  bool send_frame(const Frame &frame);
  bool tx_finished();
  bool tx_succeeded() const { return tx_succeeded_; }
  void begin_rx();

 protected:
  static void IRAM_ATTR sync_interrupt_(RedLinkRadio *radio);
  bool select_();
  bool wait_ready_();
  bool strobe_(uint8_t command);
  bool read_register_(uint8_t address, uint8_t &value);
  bool read_stable_status_(uint8_t address, uint8_t &value);
  bool write_register_(uint8_t address, uint8_t value);
  bool read_fifo_(uint8_t *data, size_t size);
  bool write_fifo_(const uint8_t *data, size_t size);
  bool wait_state_(uint8_t state);
  bool enter_idle_();
  void disarm_sync_();
  void fail_radio_(const char *reason);
  virtual void on_radio_failure_() {}
  bool listen_slot_(uint8_t slot);
  bool receiving_() const { return rx_active_ || sync_pending_; }

  InternalGPIOPin *gdo0_pin_{nullptr};
  InternalGPIOPin *miso_pin_{nullptr};
  uint32_t frequency_{904195310};
  uint16_t sync_word_{0x636e};
  uint8_t chip_version_{0};
  uint8_t receive_slot_{0};
  bool initialized_{false};
  binary_sensor::BinarySensor *radio_healthy_{nullptr};
  bool receive_only_{true};
  bool transmit_enabled_{false};
  bool rx_running_{false};
  bool rx_active_{false};
  bool tx_active_{false};
  bool tx_succeeded_{false};
  uint32_t tx_started_us_{0};
  uint32_t tx_timeout_us_{0};
  uint32_t rx_sync_end_us_{0};
  uint32_t rx_last_progress_us_{0};
  size_t rx_expected_{0};
  Frame rx_frame_{};

  // The GPIO ISR runs on the setup/main-loop core. Main-context handoffs use InterruptLock.
  volatile bool sync_armed_{false};
  volatile bool sync_pending_{false};
  volatile uint32_t sync_end_us_{0};
};

}  // namespace esphome::redlink
