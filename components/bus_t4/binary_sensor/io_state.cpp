#include "io_state.h"

#include "esphome/core/log.h"

namespace esphome::bus_t4 {

static const char *TAG = "bus_t4.io";

void BusT4IoState::on_value_(const uint8_t *payload, uint8_t len) {
  for (auto &bit : bits_) {
    // A shorter reply than the layout expects leaves that bit unknown rather than off
    if (bit.byte < len)
      bit.sensor->publish_state((payload[bit.byte] & bit.mask) != 0);
  }
}

void BusT4IoState::dump_config() {
  ESP_LOGCONFIG(TAG, "Bus T4 I/O diagnostics (register 0x%02X):", register_);
  for (auto &bit : bits_)
    LOG_BINARY_SENSOR("  ", "Bit", bit.sensor);
}

}  // namespace esphome::bus_t4
