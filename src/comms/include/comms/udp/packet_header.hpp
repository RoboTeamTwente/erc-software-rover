#pragma once
#include <cstdint>

#pragma pack(push, 1)
struct PacketHeader {
  uint16_t msg_type;        // network byte order
  uint16_t payload_length;  // network byte order
  uint32_t seq;             // network byte order
};
#pragma pack(pop)

static_assert(sizeof(PacketHeader) == 8, "PacketHeader must be exactly 8 bytes");