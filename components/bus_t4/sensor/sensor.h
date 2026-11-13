#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "../param.h"

namespace esphome::bus_t4 {

// Any control unit register, read-only, as a number
class BusT4RegisterSensor final : public sensor::Sensor, public Component, public BusT4Param {
 public:
  void set_window(uint8_t offset, uint8_t length) {
    offset_ = offset;
    length_ = length;
  }

  void loop() override { poll_(); }
  void dump_config() override;

 protected:
  void on_value_(const uint8_t *payload, uint8_t len) override;

  uint8_t offset_{0};
  uint8_t length_{0};  // 0 takes the rest of the reply
};

}  // namespace esphome::bus_t4
