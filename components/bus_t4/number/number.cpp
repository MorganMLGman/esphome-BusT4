#include "number.h"

#include "esphome/core/log.h"

namespace esphome::bus_t4 {

static const char *TAG = "bus_t4.number";

void BusT4ConfigNumber::dump_config() {
  LOG_NUMBER("", "Bus T4 Config Number", this);
  ESP_LOGCONFIG(TAG, "  Register: 0x%02X", register_);
}

}  // namespace esphome::bus_t4
