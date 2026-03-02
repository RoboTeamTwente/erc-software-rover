#pragma once
#include <cstdint>
#include <cstddef>

class Handler {
public:
  virtual ~Handler() = default;
  virtual void handle(const uint8_t* data, std::size_t length) = 0;
};