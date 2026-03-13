#pragma once

#include "components/common/envelope.pb.h"

class Handler {
public:
  virtual ~Handler() = default;
  virtual void handle(const PBEnvelope& envelope) = 0;
};
