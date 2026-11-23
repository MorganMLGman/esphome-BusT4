#pragma once

#include <cstdint>
#include <cstring>

#include "t4_packet.h"

// Byte-wise receiver for the line format [BREAK] SYNC SIZE DATA[N] SIZE. The size is sent twice
// and the checksums live inside DATA.
//
// A sender that aborts a frame and sends it again leaves the restart inside the broken frame's
// buffer. A bad frame is therefore replayed from its first later SYNC, so the restart is not lost.
class T4RxParser {
 public:
  enum class Event : uint8_t { NONE, PACKET, HEADER_CHECKSUM, PAYLOAD_CHECKSUM, TRAILING_SIZE };

  // Next byte off the wire
  Event feed(uint8_t byte) { return process(byte, false); }

  // Bytes of a bad frame still to be replayed; drain with step_pending() after every feed()
  bool has_pending() const { return pending_pos_ < pending_len_; }
  uint8_t pending_count() const { return pending_len_ - pending_pos_; }
  Event step_pending() { return process(pending_[pending_pos_++], true); }

  // Between frames
  bool idle() const { return state_ == WAIT_SYNC; }
  void reset() {
    state_ = WAIT_SYNC;
    pending_len_ = pending_pos_ = 0;
  }

  // The frame in progress, or the one the last event reported
  const T4Packet &packet() const { return packet_; }
  uint8_t received() const { return state_ == DATA ? packet_.size : 0; }
  uint8_t expected_size() const { return expected_size_; }

  // The two values an error event compared
  uint8_t expected() const { return expected_; }
  uint8_t got() const { return got_; }

 private:
  enum State : uint8_t { WAIT_SYNC, SIZE, DATA, TRAILING_SIZE };

  Event process(uint8_t byte, bool replayed) {
    switch (state_) {
      case WAIT_SYNC:
        if (byte == T4_SYNC)
          state_ = SIZE;
        return Event::NONE;

      case SIZE:
        if (byte > 0 && byte <= MAX_FRAME) {
          expected_size_ = byte;
          packet_.size = 0;
          state_ = DATA;
        } else {
          state_ = byte == T4_SYNC ? SIZE : WAIT_SYNC;
        }
        return Event::NONE;

      case DATA:
        packet_.data[packet_.size++] = byte;
        if (packet_.size == expected_size_)
          state_ = TRAILING_SIZE;
        return Event::NONE;

      case TRAILING_SIZE: {
        state_ = WAIT_SYNC;
        const Event event = validate(byte);
        if (event == Event::PACKET)
          return event;
        resync(byte);
        // A replayed frame was already reported when it was first received
        return replayed ? Event::NONE : event;
      }
    }
    return Event::NONE;
  }

  Event validate(uint8_t trailing) {
    if (trailing != expected_size_) {
      expected_ = expected_size_;
      got_ = trailing;
      return Event::TRAILING_SIZE;
    }
    // CRC1: XOR of bytes 0-5 is byte 6
    const uint8_t header = packet_.checksum(0, 6);
    if (header != packet_.data[6]) {
      expected_ = header;
      got_ = packet_.data[6];
      return Event::HEADER_CHECKSUM;
    }
    // CRC2: XOR of bytes 7 to size-2 is the last byte
    if (packet_.size >= 9) {
      const uint8_t payload = packet_.checksum(7, packet_.size - 8);
      if (payload != packet_.data[packet_.size - 1]) {
        expected_ = payload;
        got_ = packet_.data[packet_.size - 1];
        return Event::PAYLOAD_CHECKSUM;
      }
    }
    return Event::PACKET;
  }

  // Queue everything after the broken frame's own SYNC, from the first later 0x55 on. It may be
  // data, so a replayed frame has to pass both checksums to count.
  void resync(uint8_t trailing) {
    uint8_t tail[MAX_FRAME + 1];
    uint8_t tail_len = 0;
    uint8_t first = 0;
    while (first < packet_.size && packet_.data[first] != T4_SYNC)
      first++;
    for (uint8_t i = first; i < packet_.size; i++)
      tail[tail_len++] = packet_.data[i];
    if (first < packet_.size || trailing == T4_SYNC)
      tail[tail_len++] = trailing;
    if (tail_len == 0)
      return;

    // Bytes still queued from an earlier replay follow the new ones
    uint8_t rest = pending_len_ - pending_pos_;
    if (tail_len + rest > sizeof(pending_))
      rest = sizeof(pending_) - tail_len;
    memmove(pending_ + tail_len, pending_ + pending_pos_, rest);
    memcpy(pending_, tail, tail_len);
    pending_pos_ = 0;
    pending_len_ = tail_len + rest;
  }

  static constexpr uint8_t MAX_FRAME = 60;

  State state_{WAIT_SYNC};
  T4Packet packet_;
  uint8_t expected_size_{0};
  uint8_t expected_{0};
  uint8_t got_{0};
  // A replay never grows: each one starts after the bytes it consumed
  uint8_t pending_[2 * MAX_FRAME + 4];
  uint8_t pending_pos_{0};
  uint8_t pending_len_{0};
};
