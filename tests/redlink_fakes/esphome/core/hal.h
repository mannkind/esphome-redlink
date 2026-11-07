#pragma once

#include <cstdint>

namespace esphome {
uint32_t micros();
uint32_t millis();
void delayMicroseconds(uint32_t duration);
}  // namespace esphome
