#ifndef _LINKCABLE_PASSIVE_H_INCLUDE_
#define _LINKCABLE_PASSIVE_H_INCLUDE_

typedef void (*linkcable_handler_t) (void);

// Initialization for the handle_time_request
void init_time_request_handler(void);
// Multi-core compatible function for handling last_transfer_time requests
void handle_time_request(void);

// Receive the data from the linkcable
uint32_t linkcable_passive_receive(void);
// Prepare the data to send through the linkcable, for the next transfer
void linkcable_passive_send(uint32_t data);
// Make sure the linkcable is sending/receiving the newest data
void linkcable_passive_flush(void);
// Change the linkcable's mode, if 32 bits is not available,
// set LIBMOBILE_ENABLE_NO32BIT and dummy this function out
void linkcable_passive_set_is_32(bool is_32);
// Disable the linkcable. If in a multi-core setup,
// synchronization must be guaranteed by the caller,
// after the execution of this function
void linkcable_passive_disable(void);
void linkcable_passive_enable(void);
bool linkcable_passive_is_enabled(void);
void linkcable_passive_reset(bool re_enable);
// Initialize the linkcable with the function which will handle
// incoming and outgoing data
void linkcable_passive_init(linkcable_handler_t onReceive, uint wanted_pio, uint sm, uint num_bits, uint pin_sck, uint pin_sin, uint pin_sout);
bool can_disable_linkcable_passive_handler(void);
// Debug function
void print_last_linkcable_passive(void);

#endif
