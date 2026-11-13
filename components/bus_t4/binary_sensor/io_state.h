#pragma once

#include <vector>
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/core/component.h"
#include "../param.h"

namespace esphome::bus_t4 {

// The control unit's I/O diagnostics register (0xD1), one binary sensor per selected bit
class BusT4IoState final : public Component, public BusT4Param {
 public:
  BusT4IoState() { set_register(INF_IO); }

  void add_bit(binary_sensor::BinarySensor *sensor, uint8_t byte, uint8_t mask) { bits_.push_back({sensor, byte, mask}); }

  void loop() override { poll_(); }
  void dump_config() override;

 protected:
  void on_value_(const uint8_t *payload, uint8_t len) override;

  struct Bit {
    binary_sensor::BinarySensor *sensor;
    uint8_t byte;
    uint8_t mask;
  };
  std::vector<Bit> bits_;
};

}  // namespace esphome::bus_t4
