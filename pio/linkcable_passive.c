// Implementation-specific includes
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "time_defs.h"
#include "linkcable_passive.pio.h"

// Regular includes
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "linkcable_passive.h"
#include "useful_qualifiers.h"
#include "sync.h"

// Regular defines
#define LINKCABLE_CAN_DISABLE_TIMEOUT SEC(1)

#define LOG_BUFFER_SIZE 0x2000
#define TIMEFRAMES_BUFFER_SIZE 0x1000

#define DEFAULT_SAVED_BITS 8
#define MAX_POSSIBLE_BITS 32

typedef uint16_t timeframes_t;

static user_time_t get_last_transfer_time(void);

static PIO curr_pio = pio0;
static uint curr_sm = 0;
static uint curr_pin_sck = 0;
static uint curr_pin_sin = 0;
static uint curr_pin_sout = 0;
static linkcable_handler_t linkcable_irq_handler = NULL;
static uint32_t linkcable_pio_initial_pc = 0;
uint8_t saved_bits = DEFAULT_SAVED_BITS;
static user_time_t saved_time = 0;
bool is_enabled = false;
bool is_linkcable_ready = false;
user_time_t last_transfer_time = 0;
user_time_t shared_last_transfer_time = 0;
sync_t ack_time_request;

static uint16_t patched_instructions_sck[64];
static struct pio_program linkcable_sm_passive_real;

void init_time_request_handler(void) {
    init_sync(&ack_time_request);
}

void TIME_SENSITIVE(handle_time_request)(void) {   
    if(is_sync_req(&ack_time_request)) {
        shared_last_transfer_time = last_transfer_time;
        ack_sync_req(&ack_time_request);
    }
}

static user_time_t get_last_transfer_time(void) {
    if(wait_for_sync(&ack_time_request))
        return shared_last_transfer_time;
    return last_transfer_time;
}

static void TIME_SENSITIVE(linkcable_isr)(void) {
    user_time_t curr_time = TIME_FUNCTION;
    last_transfer_time = curr_time;
    if (linkcable_irq_handler) linkcable_irq_handler();
    if (pio_interrupt_get(curr_pio, 0)) pio_interrupt_clear(curr_pio, 0);
}

bool linkcable_passive_is_enabled(void) {
    return is_enabled;
}

bool can_disable_linkcable_passive_handler(void) {
    if(!is_enabled)
        return true;
    user_time_t old_time = get_last_transfer_time();
    user_time_t curr_time = TIME_FUNCTION;
    if((curr_time - old_time) >= LINKCABLE_CAN_DISABLE_TIMEOUT)
        return true;
    return false;
}

void print_last_linkcable_passive(void) {
}

uint32_t TIME_SENSITIVE(linkcable_passive_receive)(void) {
    uint32_t retval = (pio_sm_get(curr_pio, curr_sm) & ((1 << saved_bits) - 1));
    return retval;
}

void TIME_SENSITIVE(linkcable_passive_send)(uint32_t data) {
    uint32_t sendval = (data << (32 - saved_bits));
    pio_sm_put(curr_pio, curr_sm, sendval);
}

void TIME_SENSITIVE(linkcable_passive_flush)(void) {
    pio_sm_clear_fifos(curr_pio, curr_sm);
}

void linkcable_passive_enable(void) {
    while(!is_linkcable_ready);
    is_enabled = true;
    linkcable_sm_passive_activate(curr_pio, curr_sm, linkcable_pio_initial_pc, saved_bits, curr_pin_sck, curr_pin_sin, curr_pin_sout);
}

void linkcable_passive_disable(void) {
    linkcable_passive_reset(false);
}

void linkcable_passive_reset(bool re_enable) {
    pio_sm_set_enabled(curr_pio, curr_sm, false);
    is_enabled = false;
    pio_sm_clear_fifos(curr_pio, curr_sm);
    pio_sm_restart(curr_pio, curr_sm);
    pio_sm_clkdiv_restart(curr_pio, curr_sm);
    pio_sm_exec(curr_pio, curr_sm, pio_encode_jmp(linkcable_pio_initial_pc));
    if(re_enable)
        linkcable_passive_enable();
}

void linkcable_passive_set_is_32(bool is_32) {
    if(is_32)
        saved_bits = 32;
    else
        saved_bits = 8;
    linkcable_sm_passive_select_mode(curr_pio, curr_sm, saved_bits);
}

void linkcable_passive_init(irq_handler_t onDataReceive, uint wanted_pio, uint sm, uint num_bits, uint pin_sck, uint pin_sin, uint pin_sout) {
    curr_pio = pio0;
    int irq_pio = PIO0_IRQ_0;
    if(wanted_pio == 1) {
        curr_pio = pio1;
        irq_pio = PIO1_IRQ_0;
    }
    curr_sm = sm;
    curr_pin_sck = pin_sck;
    curr_pin_sin = pin_sin;
    curr_pin_sout = pin_sout;
    linkcable_sm_passive_real = linkcable_sm_passive_patch_gpio_sck_and_load(curr_pin_sck, patched_instructions_sck);
    linkcable_sm_passive_program_init_partial(curr_pio, curr_sm, linkcable_pio_initial_pc = pio_add_program(curr_pio, &linkcable_sm_passive_real), MAX_POSSIBLE_BITS, curr_pin_sck, curr_pin_sin, curr_pin_sout);
    pio_enable_sm_mask_in_sync(curr_pio, (1u << curr_sm));

    if (onDataReceive) {
        linkcable_irq_handler = onDataReceive;
        pio_set_irq0_source_enabled(curr_pio, pis_interrupt0, true);
        irq_set_exclusive_handler(irq_pio, linkcable_isr);
        irq_set_enabled(irq_pio, true);
    }
    is_linkcable_ready = true;
}
