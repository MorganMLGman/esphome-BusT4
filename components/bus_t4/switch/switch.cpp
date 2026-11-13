#include "switch.h"

#include "esphome/core/log.h"

namespace esphome::bus_t4 {

static const char *TAG = "bus_t4.switch";

void BusT4ConfigSwitch::dump_config() {
  LOG_SWITCH("", "Bus T4 Config Switch", this);
  ESP_LOGCONFIG(TAG, "  Register: 0x%02X", register_);
}

}  // namespace esphome::bus_t4
