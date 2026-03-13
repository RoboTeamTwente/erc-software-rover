#pragma once
#include <cstdint>

namespace comms::udp {

inline uint8_t clamp_u8(int v) {
  if (v < 0) return 0;
  if (v > 255) return 255;
  return static_cast<uint8_t>(v);
}

} // namespace comms::udp
