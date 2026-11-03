#ifndef IO_BUFFER_H_
#define IO_BUFFER_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

uint8_t get_data_out(bool* success);
uint32_t set_data_out(const uint8_t* buffer, uint32_t size, uint32_t pos);
uint32_t get_data_in(void);
void set_data_in(uint8_t* buffer, uint32_t size);
uint32_t available_data_out(void);
uint32_t available_data_in(void);
uint32_t get_data_out_checksum(void);
uint32_t get_data_in_checksum(void);
void reset_data_out(void);
void reset_data_in(void);

#endif /* IO_BUFFER_H_ */
