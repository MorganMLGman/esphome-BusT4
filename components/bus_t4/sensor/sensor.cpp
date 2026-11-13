#include "sensor.h"

#include "esphome/core/log.h"

namespace esphome::bus_t4 {

static const char *TAG = "bus_t4.sensor";

void BusT4RegisterSensor::on_value_(const uint8_t *payload, uint8_t len) {
  if (offset_ >= len) {
    ESP_LOGW(TAG, "Register 0x%02X: offset %u is past the %u byte(s) of the reply", register_, offset_, len);
    return;
  }
  const uint8_t available = len - offset_;
  uint8_t count = length_ == 0 ? available : length_;
  if (count > available) {
    ESP_LOGW(TAG, "Register 0x%02X: %u byte(s) asked for, the reply has %u after offset %u", register_, count,
             available, offset_);
    count = available;
  }
  publish_state(static_cast<float>(t4_be_value(payload + offset_, count)));
}

void BusT4RegisterSensor::dump_config() {
  LOG_SENSOR("", "Bus T4 Register Sensor", this);
  ESP_LOGCONFIG(TAG, "  Register: 0x%02X", register_);
  if (offset_ != 0 || length_ != 0) {
    ESP_LOGCONFIG(TAG, "  Offset: %u, length: %u (0 = rest of the reply)", offset_, length_);
  }
}

}  // namespace esphome::bus_t4
