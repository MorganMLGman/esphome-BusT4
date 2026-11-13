#pragma once

#include "esphome/components/switch/switch.h"
#include "esphome/core/component.h"
#include "../param.h"

namespace esphome::bus_t4 {

// An on/off controller setting whose state is what the controller reports, not what was sent
class BusT4ConfigSwitch final : public switch_::Switch, public Component, public BusT4Param {
 public:
  void loop() override { poll_(); }
  void dump_config() override;

 protected:
  void write_state(bool state) override { write_(state ? 1 : 0); }
  void on_value_(const uint8_t *payload, uint8_t len) override { publish_state(t4_be_value(payload, len) != 0); }
};

}  // namespace esphome::bus_t4
