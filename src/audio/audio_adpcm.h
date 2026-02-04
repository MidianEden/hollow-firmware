#pragma once

#include <Arduino.h>
#include <cstddef>
#include <cstdint>

void ima_reset_state();
size_t ima_encode_block(const int16_t *pcm, size_t pcmCount, uint8_t *out);
