#include "io_buffer.h"

#define OUT_BUFFER_SIZE 0x400
#define IN_BUFFER_SIZE 0x400

struct io_buffer {
    uint8_t *p;
    uint32_t size;
    uint32_t pos_inside;
    uint32_t pos_outside;
};

uint8_t buffer_in[IN_BUFFER_SIZE];
uint8_t buffer_out[OUT_BUFFER_SIZE];

struct io_buffer b_in = {buffer_in, IN_BUFFER_SIZE, 0, 0};
struct io_buffer b_out = {buffer_out, OUT_BUFFER_SIZE, 0, 0};

static uint8_t _get_data_out(bool* success, struct io_buffer* b) {
    *success = 0;
    uint32_t data = 0;
    if(b->pos_inside != b->pos_outside) {
        data = b->p[b->pos_outside++];
        b->pos_outside %= b->size;
        *success = 1;
    }
    return data;
}

uint8_t get_data_out(bool* success) {
    return _get_data_out(success, &b_out);
}

static uint32_t _set_data_out(const uint8_t* buffer, uint32_t size, uint32_t pos, struct io_buffer* b) {
    for(uint32_t i = pos; i < size; i++) {
        if(((b->pos_inside + 1) % b->size) == b->pos_outside)
                return i;
        b->p[b->pos_inside++] = buffer[i];
        b->pos_inside %= b->size;
    }
    return size;
}

uint32_t set_data_out(const uint8_t* buffer, uint32_t size, uint32_t pos) {
    return _set_data_out(buffer, size, pos, &b_out);
}

static uint32_t _available_data(struct io_buffer* b, size_t start, size_t end) {
    uint32_t result = end;
    if(end < start)
        result += b->size;
    result -= start;
    return result;
}

uint32_t available_data_out(void) {
    return _available_data(&b_out, b_out.pos_outside, b_out.pos_inside);
}

static uint32_t _get_data_in(struct io_buffer* b) {
    uint32_t data;
    if(b->pos_inside == b->pos_outside)
        data = -1;
    else {
        data = b->p[b->pos_inside];
        b->pos_inside = (b->pos_inside + 1) % b->size;
    }
    return data;
}

uint32_t get_data_in(void) {
    return _get_data_in(&b_in);
}

static void _set_data_in(uint8_t* buffer, uint32_t size, struct io_buffer* b) {
    for(int i = 0; i < size; i++) {
        b->p[b->pos_outside++] = buffer[i];
        b->pos_outside %= b->size;
    }
}

void set_data_in(uint8_t* buffer, uint32_t size) {
    _set_data_in(buffer, size, &b_in);
}

uint32_t available_data_in(void) {
    return _available_data(&b_in, b_in.pos_inside, b_in.pos_outside);
}

uint32_t _get_data_checksum(struct io_buffer* b, size_t start, size_t end) {
    uint32_t checksum = 0;
    size_t curr_elem_in_checksum = 0;
    size_t curr_pos = start;
    while(curr_pos != end) {
      checksum += (b->p[curr_pos++]) << (8 * curr_elem_in_checksum);
      curr_pos %= b->size;
      curr_elem_in_checksum = (curr_elem_in_checksum + 1) % sizeof(checksum);
    }
    return checksum;
}

uint32_t get_data_out_checksum(void) {
    return _get_data_checksum(&b_out, b_out.pos_outside, b_out.pos_inside);
}

uint32_t get_data_in_checksum(void) {
    return _get_data_checksum(&b_in, b_in.pos_inside, b_in.pos_outside);
}

static void _reset_data(struct io_buffer* b) {
    b->pos_inside = b->pos_outside;
}

void reset_data_out(void) {
    _reset_data(&b_out);
}

void reset_data_in(void) {
    _reset_data(&b_in);
}
