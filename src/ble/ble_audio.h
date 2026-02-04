#pragma once

#include <cstddef>
#include <cstdint>

void bleSendAudioChunk(const uint8_t *data, size_t len);
void bleSendControlMessage(const char *msg);
