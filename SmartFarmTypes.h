#pragma once
#include <Arduino.h>

enum GrowPhase_t : uint8_t {
  PHASE_1_ESTABLISH  = 0,
  PHASE_2_VEGETATIVE = 1,
  PHASE_3_FINISHING  = 2
};