#pragma once

#include "bus_t4.h"

namespace esphome::bus_t4 {

// One control unit register, read with GET and optionally written with SET. Each entity is its
// own BusT4Device so the reply reaches it directly instead of being routed through the cover.
class BusT4Param : public BusT4Device {
 public:
  void set_register(uint8_t reg) { register_ = reg; }
  // Called by register_component() for update_interval, like a PollingComponent
  void set_update_interval(uint32_t ms) { refresh_ms_ = ms; }

  void on_packet(const T4Packet &packet) override;

 protected:
  // Called from the owning entity's loop()
  void poll_();
  // Writes with the width the controller last reported, then reads the value back
  void write_(uint32_t value);
  // Big-endian payload of a successful GET reply, at least one byte
  virtual void on_value_(const uint8_t *payload, uint8_t len) = 0;

  uint8_t register_{0};

 private:
  void request_();

  uint32_t refresh_ms_{60000};
  uint32_t last_request_ms_{0};
  uint8_t value_len_{1};
  bool has_value_{false};
  bool unsupported_{false};
};

// Up to the first four payload bytes as one big-endian integer, the way the control unit sends them
uint32_t t4_be_value(const uint8_t *payload, uint8_t len);

}  // namespace esphome::bus_t4
