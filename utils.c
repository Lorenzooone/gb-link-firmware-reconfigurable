#include "utils.h"

uint32_t read_u32_le(const uint8_t* data, size_t pos) {
    return data[pos] | (data[pos + 1] << 8) | (data[pos + 2] << 16) | (data[pos + 3] << 24);
}

void write_u32_le(uint8_t* data, size_t pos, uint32_t value) {
    data[pos] = (value) & 0xFF;
    data[pos + 1] = (value >> 8) & 0xFF;
    data[pos + 2] = (value >> 16) & 0xFF;
    data[pos + 3] = (value >> 24) & 0xFF;
}
