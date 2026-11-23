#include "bus_t4_component.h"
#include "bus_t4.h"

#include "esphome/core/log.h"
#include "esphome/core/alloc_helpers.h"
#include "esphome/core/helpers.h"

#include "t4_packet.h"

#include <esp_rom_sys.h>

#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace esphome::bus_t4 {

static const char *TAG = "bus_t4";

// Bumped when tagging a release; checked against the tag by CI
static const char *VERSION = "2.2.1";

// Break signal baud rate: sending 0x00 at 9200 baud produces a ~1ms low pulse
// on a 19200 baud bus. Reference: pruwait/Nice_BusT4 nice-bust4.h
static constexpr uint32_t T4_BAUD_BREAK = 9200;

// Observed request-to-reply latency on a shared bus reaches ~55ms
static constexpr uint32_t BUS_IDLE_MS = 100;

// Bounded so a busy bus cannot starve the transmit path
static constexpr uint8_t RX_DRAIN_LIMIT = 64;

// A full frame takes at most ~33ms at 19200 baud and arrives without gaps, so this much
// silence mid-frame means it will not complete
static constexpr uint32_t FRAME_STALL_MS = 50;

static constexpr TickType_t TX_MIN_INTERVAL = pdMS_TO_TICKS(100);

void BusT4Component::setup() {
  rxQueue_ = xQueueCreate(32, sizeof(T4Packet));
  if (rxQueue_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create RX queue");
    this->mark_failed();
    return;
  }

  txQueue_ = xQueueCreate(32, sizeof(T4Frame));
  if (txQueue_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create TX queue");
    vQueueDelete(rxQueue_);
    rxQueue_ = nullptr;
    this->mark_failed();
    return;
  }

  if (xTaskCreate(busTaskThunk, "bus_t4", 8192, this, 10, &busTask_) != pdPASS) {
    ESP_LOGE(TAG, "Failed to create bus task");
    this->mark_failed();
    return;
  }

  // Cache UART port number for direct baud rate changes during break signal.
  auto *idf_uart = static_cast<uart::IDFUARTComponent *>(parent_);
  uart_num_ = static_cast<uart_port_t>(idf_uart->get_hw_serial_number());
}

void BusT4Component::loop() {
  // Process received packets and dispatch to registered devices
  T4Packet packet;
  while (xQueueReceive(rxQueue_, &packet, 0)) {
    // Ignore packets FROM ourselves (TX echo on half-duplex bus)
    if (packet.header.from == address_) {
      ESP_LOGV(TAG, "Ignoring TX echo");
      continue;
    }

    // Log foreign destinations without dropping them yet
    if (!t4_addressed_to(packet.header.to, address_)) {
      ESP_LOGV(TAG, "Packet addressed to 0x%02X.%02X, not us: %s",
               packet.header.to.address, packet.header.to.endpoint,
               format_hex_pretty(packet.data, packet.size).c_str());
    }

    // Dispatch to all registered devices
    for (auto *device : devices_) {
      device->on_packet(packet);
    }
  }

  const uint32_t errors = rx_errors_;
  if (bus_errors_sensor_ != nullptr && (!bus_errors_sensor_->has_state() || errors != published_errors_)) {
    published_errors_ = errors;
    bus_errors_sensor_->publish_state(errors);
  }

  if (last_error_sensor_ != nullptr) {
    if (!last_error_sensor_->has_state()) last_error_sensor_->publish_state("none");

    char text[sizeof(last_error_)];
    bool pending;
    {
      LockGuard guard(last_error_lock_);
      pending = last_error_pending_;
      if (pending) {
        memcpy(text, last_error_, sizeof(text));
        last_error_pending_ = false;
      }
    }
    if (pending) last_error_sensor_->publish_state(text);
  }
}

void BusT4Component::note_rx_error(const uint8_t *data, size_t len, const char *fmt, ...) {
  const uint32_t number = ++rx_errors_;
  if (last_error_sensor_ == nullptr) return;

  char what[96];
  va_list args;
  va_start(args, fmt);
  vsnprintf(what, sizeof(what), fmt, args);
  va_end(args);

  char hex[format_hex_pretty_size(sizeof(T4Packet::data))];
  format_hex_pretty_to(hex, sizeof(hex), data, len, '.');

  // The number keeps two identical frames distinct for Home Assistant's history
  LockGuard guard(last_error_lock_);
  // Home Assistant refuses states over 255 characters, so the frame is cut to its first 40 bytes
  snprintf(last_error_, sizeof(last_error_), "#%u %s: %.119s", static_cast<unsigned>(number), what, hex);
  last_error_pending_ = true;
}

void BusT4Component::dump_config() {
  ESP_LOGCONFIG(TAG, "BusT4:");
  ESP_LOGCONFIG(TAG, "  Version: %s", VERSION);
  ESP_LOGCONFIG(TAG, "  Address: 0x%02X%02X", address_.address, address_.endpoint);
}

void BusT4Component::busTask() {
  T4Packet packet;
  uint8_t expected_size = 0;
  // Protocol format: [BREAK] SYNC SIZE DATA[N] SIZE
  // The size byte is sent twice (start and end), no separate checksum byte.
  // Internal checksums are within the DATA portion.
  enum { WAIT_SYNC = 0, SIZE, DATA, TRAILING_SIZE } rx_state = WAIT_SYNC;
  uint32_t last_rx_ms = 0;
  TickType_t last_tx_time = 0;

  // Timing of the frame being received, reported with its errors so a collision with our own
  // transmission can be told apart from a stalled task or a sender that paused mid-frame
  uint32_t last_tx_end_ms = 0;
  uint32_t frame_start_ms = 0;
  uint32_t max_gap_ms = 0;   // longest silence between two bytes of the frame
  uint32_t max_lag_ms = 0;   // longest pass of this task while the frame was open
  uint32_t last_pass_ms = millis();
  auto describe_timing = [&](char *out, size_t size) {
    snprintf(out, size, "tx+%ums dur %ums gap %ums lag %ums",
             static_cast<unsigned>(frame_start_ms - last_tx_end_ms),
             static_cast<unsigned>(last_rx_ms - frame_start_ms), static_cast<unsigned>(max_gap_ms),
             static_cast<unsigned>(max_lag_ms));
  };
  char timing[48];

  for (;;) {
    uint8_t byte;
    uint8_t drained = 0;
    const uint32_t pass_ms = millis();
    if (rx_state != WAIT_SYNC)
      max_lag_ms = std::max(max_lag_ms, pass_ms - last_pass_ms);
    last_pass_ms = pass_ms;
    while (drained < RX_DRAIN_LIMIT && parent_->available() && parent_->read_byte(&byte) == true) {
      drained++;
      const uint32_t now = millis();
      if (rx_state != WAIT_SYNC)
        max_gap_ms = std::max(max_gap_ms, now - last_rx_ms);
      last_rx_ms = now;
      switch (rx_state) {
        case WAIT_SYNC:
          // Wait for SYNC byte (0x55), ignore break bytes (0x00) and others
          if (byte == T4_SYNC) {
            rx_state = SIZE;
            frame_start_ms = now;
            max_gap_ms = 0;
            max_lag_ms = 0;
          }
          break;

        case SIZE:
          if (byte > 0 && byte <= 60) {
            expected_size = byte;
            packet.size = 0;
            rx_state = DATA;
          } else {
            // Invalid size, go back to waiting for sync
            rx_state = WAIT_SYNC;
          }
          break;

        case DATA:
          packet.data[packet.size++] = byte;
          if (packet.size == expected_size)
            rx_state = TRAILING_SIZE;
          break;

        case TRAILING_SIZE:
          // Verify trailing size matches
          if (byte == expected_size) {
            // Verify header checksum (CRC1): XOR of bytes 0-5 should equal byte 6
            uint8_t header_check = packet.checksum(0, 6);
            if (header_check != packet.data[6]) {
              ESP_LOGW(TAG, "Header checksum mismatch: expected 0x%02X, got 0x%02X: %s",
                       header_check, packet.data[6],
                       format_hex_pretty(packet.data, packet.size).c_str());
              describe_timing(timing, sizeof(timing));
              note_rx_error(packet.data, packet.size, "header checksum, expected 0x%02X got 0x%02X (%s)",
                            header_check, packet.data[6], timing);
              rx_state = WAIT_SYNC;
              break;
            }

            // Verify payload checksum (CRC2): XOR of bytes 7 to size-2 should equal byte size-1
            if (packet.size >= 9) {  // Minimum: 7 header + 1 payload + 1 CRC2
              uint8_t payload_check = packet.data[7];
              for (size_t i = 8; i < packet.size - 1; i++) {
                payload_check ^= packet.data[i];
              }
              if (payload_check != packet.data[packet.size - 1]) {
                ESP_LOGW(TAG, "Payload checksum mismatch: expected 0x%02X, got 0x%02X: %s",
                         payload_check, packet.data[packet.size - 1],
                         format_hex_pretty(packet.data, packet.size).c_str());
                describe_timing(timing, sizeof(timing));
                note_rx_error(packet.data, packet.size, "payload checksum, expected 0x%02X got 0x%02X (%s)",
                              payload_check, packet.data[packet.size - 1], timing);
                rx_state = WAIT_SYNC;
                break;
              }
            }

            // Both checksums valid - accept packet
            ESP_LOGV(TAG, "Received packet: %s (%d bytes)",
                     format_hex_pretty(packet.data, packet.size).c_str(), packet.size);
            if (!xQueueSend(rxQueue_, &packet, pdMS_TO_TICKS(100))) {
              ESP_LOGW(TAG, "RX queue full, dropping packet");
              note_rx_error(packet.data, packet.size, "RX queue full");
            }
          } else {
            ESP_LOGW(TAG, "Trailing size mismatch: expected 0x%02X, got 0x%02X", expected_size, byte);
            describe_timing(timing, sizeof(timing));
            note_rx_error(packet.data, packet.size, "trailing size, expected 0x%02X got 0x%02X (%s)", expected_size,
                          byte, timing);
          }
          rx_state = WAIT_SYNC;
          break;
      }
    }

    // A truncated frame would otherwise hold rx_state forever and, with it, every transmit -
    // including the polls the controller needs before it says anything again
    if (rx_state != WAIT_SYNC && millis() - last_rx_ms >= FRAME_STALL_MS) {
      ESP_LOGW(TAG, "Frame stalled after %u of %u bytes, resyncing",
               static_cast<unsigned>(packet.size), static_cast<unsigned>(expected_size));
      describe_timing(timing, sizeof(timing));
      note_rx_error(packet.data, packet.size, "stalled after %u of %u bytes (%s)",
                    static_cast<unsigned>(packet.size), static_cast<unsigned>(expected_size), timing);
      rx_state = WAIT_SYNC;
    }

    // Transmit only between frames, and only once the bus has gone quiet
    if (rx_state == WAIT_SYNC && millis() - last_rx_ms >= BUS_IDLE_MS) {
      T4Frame frame;
      if (xQueueReceive(txQueue_, &frame, 0)) {
        TickType_t elapsed = xTaskGetTickCount() - last_tx_time;
        if (elapsed < TX_MIN_INTERVAL) {
          vTaskDelay(TX_MIN_INTERVAL - elapsed);
        }

        if (frame.raw) {
          ESP_LOGV(TAG, "Sending raw: %s", format_hex_pretty(frame.data, frame.size).c_str());
        } else {
          ESP_LOGV(TAG, "Sending packet: %s",
                   format_hex_pretty(frame.data + 2, frame.size - 3).c_str());
        }
        send_break();
        parent_->write_array(frame.data, frame.size);
        parent_->flush();

        last_tx_time = xTaskGetTickCount();
        last_tx_end_ms = millis();
      }
    }

    vTaskDelay(1);
  }

  busTask_ = nullptr;
  vTaskDelete(nullptr);
}

bool BusT4Component::queue_frame(const T4Frame &frame, TickType_t xTicksToWait) {
  if (txQueue_ == nullptr)
    return false;
  // Callers fire and forget, so this is the only trace a dropped command leaves
  if (!xQueueSend(txQueue_, &frame, xTicksToWait)) {
    ESP_LOGW(TAG, "TX queue full, dropping frame");
    return false;
  }
  return true;
}

bool BusT4Component::write(T4Packet *packet, TickType_t xTicksToWait) {
  if (packet == nullptr || packet->size == 0 || packet->size > sizeof(T4Packet::data))
    return false;

  T4Frame frame;
  frame.data[0] = T4_SYNC;
  frame.data[1] = packet->size;
  std::copy_n(packet->data, packet->size, frame.data + 2);
  frame.data[packet->size + 2] = packet->size;
  frame.size = packet->size + 3;
  return queue_frame(frame, xTicksToWait);
}

void BusT4Component::write_raw(const uint8_t *data, size_t len) {
  // User-provided bytes are already framed, so they go out verbatim
  if (len == 0 || len > sizeof(T4Frame::data)) {
    ESP_LOGW(TAG, "Raw frame of %u bytes rejected", static_cast<unsigned>(len));
    return;
  }

  T4Frame frame;
  frame.raw = true;
  frame.size = static_cast<uint8_t>(len);
  std::copy_n(data, len, frame.data);
  queue_frame(frame, 0);
}

void BusT4Component::send_break() {
  // BusT4 protocol requires a break signal (~1ms low pulse) before each packet.
  // Achieved by sending 0x00 at a lower baud rate (9200 instead of 19200).
  // Reference: pruwait/Nice_BusT4 send_array_cmd() implementation.
  if (uart_num_ < UART_NUM_MAX) {
    uint32_t work_baud = parent_->get_baud_rate();
    uart_set_baudrate(uart_num_, T4_BAUD_BREAK);
    parent_->write_byte(T4_BREAK);
    parent_->flush();
    esp_rom_delay_us(100);
    uart_set_baudrate(uart_num_, work_baud);
  } else {
    // UART port not cached — fall back to simple break byte
    parent_->write_byte(T4_BREAK);
    parent_->flush();
  }
}

} // namespace esphome::bus_t4
