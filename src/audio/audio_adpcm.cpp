#include "audio_adpcm.h"

static const int16_t ima_step_table[89] = {
     7, 8, 9, 10, 11, 12, 13, 14,
    16, 17, 19, 21, 23, 25, 28, 31,
    34, 37, 41, 45, 50, 55, 60, 66,
    73, 80, 88, 97, 107, 118, 130, 143,
   157, 173, 190, 209, 230, 253, 279, 307,
   337, 371, 408, 449, 494, 544, 598, 658,
   724, 796, 876, 963, 1060, 1166, 1282, 1411,
  1552, 1707, 1878, 2066, 2272, 2499, 2749, 3024,
  3327, 3660, 4026, 4428, 4871, 5358, 5894, 6484,
  7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
 15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794,
 32767
};

static const int8_t ima_index_table[16] = {
   -1, -1, -1, -1, 2, 4, 6, 8,
   -1, -1, -1, -1, 2, 4, 6, 8
};

static int16_t ima_pred = 0;
static int     ima_index = 0;

void ima_reset_state() {
    ima_pred = 0;
    ima_index = 0;
}

size_t ima_encode_block(const int16_t *pcm, size_t pcmCount, uint8_t *out) {
    int16_t pred = ima_pred;
    int index = ima_index;

    size_t outIndex = 0;
    uint8_t nibble = 0;
    bool haveNibble = false;

    for (size_t i = 0; i < pcmCount; i++) {
        int diff = pcm[i] - pred;
        uint8_t code = 0;

        if (diff < 0) { code |= 8; diff = -diff; }

        int step = ima_step_table[index];
        int temp = step;

        if (diff >= temp) { code |= 4; diff -= temp; }
        temp >>= 1;
        if (diff >= temp) { code |= 2; diff -= temp; }
        temp >>= 1;
        if (diff >= temp) { code |= 1; }

        int delta = step >> 3;
        if (code & 4) delta += step;
        if (code & 2) delta += step >> 1;
        if (code & 1) delta += step >> 2;
        if (code & 8) delta = -delta;

        pred += delta;
        if (pred > 32767) pred = 32767;
        if (pred < -32768) pred = -32768;

        index += ima_index_table[code];
        if (index < 0) index = 0;
        if (index > 88) index = 88;

        if (!haveNibble) {
            nibble = code & 0x0F;
            haveNibble = true;
        } else {
            out[outIndex++] = (nibble | ((code & 0x0F) << 4));
            haveNibble = false;
        }
    }

    if (haveNibble) out[outIndex++] = nibble;

    ima_pred = pred;
    ima_index = index;

    return outIndex;
}
