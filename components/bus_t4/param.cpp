#include "param.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::bus_t4 {

static const char *TAG = "bus_t4.param";

// Until the first reply a lost frame should not cost a whole refresh interval
static constexpr uint32_t PARAM_RETRY_MS = 5000;

static constexpr uint8_t DMP_DATA_OFFSET = 12;

uint32_t t4_be_value(const uint8_t *payload, uint8_t len) {
  uint32_t value = 0;
  for (uint8_t i = 0; i < len && i < 4; i++)
    value = (value << 8) | payload[i];
  return value;
}

void BusT4Param::poll_() {
  if (unsupported_ || parent_ == nullptr || !parent_->controller_ready())
    return;
  const uint32_t interval = has_value_ ? refresh_ms_ : PARAM_RETRY_MS;
  if (last_request_ms_ != 0 && millis() - last_request_ms_ < interval)
    return;
  request_();
}

void BusT4Param::request_() {
  target_address_ = parent_->controller_address();
  send_info_request(FOR_CU, static_cast<T4InfoCommand>(register_));
  last_request_ms_ = millis();
}

void BusT4Param::write_(uint32_t value) {
  if (parent_ == nullptr || !parent_->controller_ready()) {
    ESP_LOGW(TAG, "Register 0x%02X: control unit not found yet, write dropped", register_);
    return;
  }
  target_address_ = parent_->controller_address();

  // DMP SET: [target] [register] [request] [offset] [length] [value, big-endian]
  uint8_t message[9] = {FOR_CU, register_, REQ_SET, 0x00, value_len_};
  for (uint8_t i = 0; i < value_len_; i++)
    message[5 + i] = static_cast<uint8_t>(value >> (8 * (value_len_ - 1 - i)));
  T4Packet packet(target_address_, parent_->get_address(), DMP, message, 5 + value_len_);
  ESP_LOGI(TAG, "Register 0x%02X: writing %u", register_, static_cast<unsigned>(value));
  write(&packet, 0);

  // The entity shows only what the controller reports back, never the value it was asked for
  has_value_ = false;
  request_();
}

void BusT4Param::on_packet(const T4Packet &packet) {
  if (packet.header.protocol != DMP || packet.header.from != target_address_)
    return;
  if (packet.size < 13 || packet.message.device != FOR_CU || packet.message.command != register_)
    return;
  // SET acknowledgements are skipped: the state comes from the GET that follows each write
  const uint8_t flags = packet.message.dmp.flags;
  if (flags != RSP_GET_COMPLETE && flags != RSP_GET_INCOMPLETE)
    return;

  if (packet.message.dmp.status != ERR_NONE) {
    // A rejected register stays rejected; polling it again would only add bus traffic
    ESP_LOGW(TAG, "Register 0x%02X: control unit rejected it (0x%02X), no longer polled", register_,
             packet.message.dmp.status);
    unsupported_ = true;
    return;
  }

  const uint8_t len = t4_dmp_payload_len(packet);
  if (len == 0)
    return;
  value_len_ = len < 4 ? len : 4;
  has_value_ = true;
  on_value_(packet.data + DMP_DATA_OFFSET, len);
}

}  // namespace esphome::bus_t4
