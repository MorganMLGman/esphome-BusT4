// Host-side check of the receive parser: g++ -std=c++17 -Wall -I components/bus_t4 tests/test_rx_parser.cpp
#include <cassert>
#include <cstdio>
#include <sstream>
#include <string>
#include <vector>

#include "t4_rx_parser.h"

using Bytes = std::vector<uint8_t>;

static Bytes hex(const std::string &text) {
  Bytes out;
  std::istringstream in(text);
  std::string token;
  while (in >> token)
    out.push_back(static_cast<uint8_t>(std::stoi(token, nullptr, 16)));
  return out;
}

struct Outcome {
  std::vector<T4RxParser::Event> events;
  std::vector<Bytes> packets;
};

// Feeds the way the bus task does: live byte first, then whatever it queued for replay
static Outcome run(const Bytes &wire) {
  T4RxParser parser;
  Outcome out;
  auto note = [&](T4RxParser::Event event) {
    if (event == T4RxParser::Event::NONE)
      return;
    out.events.push_back(event);
    if (event == T4RxParser::Event::PACKET)
      out.packets.emplace_back(parser.packet().data, parser.packet().data + parser.packet().size);
  };
  for (uint8_t byte : wire) {
    note(parser.feed(byte));
    while (parser.has_pending())
      note(parser.step_pending());
  }
  return out;
}

static Bytes concat(std::initializer_list<Bytes> parts) {
  Bytes out;
  for (const Bytes &part : parts)
    out.insert(out.end(), part.begin(), part.end());
  return out;
}

static const T4RxParser::Event PACKET = T4RxParser::Event::PACKET;
static const T4RxParser::Event TRAILING = T4RxParser::Event::TRAILING_SIZE;
static const T4RxParser::Event PAYLOAD = T4RxParser::Event::PAYLOAD_CHECKSUM;
static const T4RxParser::Event HEADER = T4RxParser::Event::HEADER_CHECKSUM;

int main() {
  // Frames captured from the bus
  const Bytes d1 = hex("50 90 00 03 08 14 DF 04 D1 19 0E 00 01 00 00 00 00 04 00 00 00 00 00 00 00 00 C7");
  const Bytes pos = hex("50 90 00 03 08 09 C2 04 11 19 03 00 01 00 02 0C");
  const Bytes d1_wire = concat({hex("00 55 1B"), d1, hex("1B")});
  const Bytes pos_wire = concat({hex("00 55 10"), pos, hex("10")});

  // A clean frame is one packet, and noise before it changes nothing
  {
    Outcome o = run(concat({hex("00 00 12 55 55"), hex("1B"), d1, hex("1B")}));
    assert(o.events == std::vector<T4RxParser::Event>{PACKET});
    assert(o.packets[0] == d1);
  }

  // last_bus_error #8: the reply stops after 13 bytes and the whole frame follows again
  {
    Bytes cut = concat({hex("00 55 1B"), Bytes(d1.begin(), d1.begin() + 13)});
    Outcome o = run(concat({cut, d1_wire}));
    assert((o.events == std::vector<T4RxParser::Event>{TRAILING, PACKET}));
    assert(o.packets[0] == d1);
  }

  // #2: only three bytes of the position reply make it out
  {
    Outcome o = run(concat({hex("00 55 10 50"), pos_wire}));
    assert((o.events == std::vector<T4RxParser::Event>{TRAILING, PACKET}));
    assert(o.packets[0] == pos);
  }

  // The restart's SIZE byte lands where the trailing size was expected and matches it
  {
    Bytes cut = concat({hex("00 55 1B"), Bytes(d1.begin(), d1.begin() + 26), hex("55")});
    Bytes rest = concat({Bytes(d1.begin(), d1.end()), hex("1B")});
    Outcome o = run(concat({cut, hex("1B"), rest}));
    assert((o.events == std::vector<T4RxParser::Event>{PAYLOAD, PACKET}));
    assert(o.packets[0] == d1);
  }

  // The restart's SYNC itself is the byte read as the trailing size
  {
    Bytes cut = concat({hex("00 55 1B"), Bytes(d1.begin(), d1.begin() + 26), hex("00")});
    Outcome o = run(concat({cut, hex("55"), hex("1B"), d1, hex("1B")}));
    assert((o.events == std::vector<T4RxParser::Event>{TRAILING, PACKET}));
    assert(o.packets[0] == d1);
  }

  // Two aborted frames in a row still end in the whole one
  {
    Bytes cut1 = concat({hex("00 55 1B"), Bytes(d1.begin(), d1.begin() + 5)});
    Bytes cut2 = concat({hex("00 55 1B"), Bytes(d1.begin(), d1.begin() + 9)});
    Outcome o = run(concat({cut1, cut2, d1_wire}));
    assert(o.packets.size() == 1 && o.packets[0] == d1);
    assert(o.events.back() == PACKET);
  }

  // A good frame after the broken one is not swallowed
  {
    Bytes cut = concat({hex("00 55 1B"), Bytes(d1.begin(), d1.begin() + 13)});
    Outcome o = run(concat({cut, d1_wire, pos_wire}));
    assert(o.packets.size() == 2 && o.packets[0] == d1 && o.packets[1] == pos);
  }

  // A corrupt frame with a 0x55 in its data reports once and invents no packet
  {
    Bytes bad = pos;
    bad[9] = 0x55;
    Outcome o = run(concat({hex("00 55 10"), bad, hex("10")}));
    assert(o.packets.empty());
    assert((o.events == std::vector<T4RxParser::Event>{PAYLOAD}));
  }

  // A header checksum failure is reported as such
  {
    Bytes bad = pos;
    bad[2] = 0x01;
    Outcome o = run(concat({hex("00 55 10"), bad, hex("10")}));
    assert((o.events == std::vector<T4RxParser::Event>{HEADER}));
  }

  // A valid frame whose data holds 0x55 stays one packet
  {
    Bytes data = pos;
    data[12] = 0x55;
    data[15] ^= 0x55 ^ pos[12];
    Outcome o = run(concat({hex("00 55 10"), data, hex("10")}));
    assert(o.events == std::vector<T4RxParser::Event>{PACKET});
  }

  puts("rx parser: all checks passed");
  return 0;
}
