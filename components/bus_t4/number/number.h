#pragma once

#include "esphome/components/number/number.h"
#include "esphome/core/component.h"
#include "../param.h"

namespace esphome::bus_t4 {

// A numeric controller setting whose state is what the controller reports, not what was sent
class BusT4ConfigNumber final : public number::Number, public Component, public BusT4Param {
 public:
  void loop() override { poll_(); }
  void dump_config() override;

 protected:
  void control(float value) override { write_(static_cast<uint32_t>(value)); }
  void on_value_(const uint8_t *payload, uint8_t len) override {
    publish_state(static_cast<float>(t4_be_value(payload, len)));
  }
};

}  // namespace esphome::bus_t4
