#ifndef UTILS_H_
#define UTILS_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

uint32_t read_u32_le(const uint8_t* data, size_t pos);
void write_u32_le(uint8_t* data, size_t pos, uint32_t value);

#endif /* UTILS_H_ */
