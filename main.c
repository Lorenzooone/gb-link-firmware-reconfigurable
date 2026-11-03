/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bsp/board.h"
#include "tusb.h"
#include "usb_descriptors.h"
#include <stdio.h>
#include <string.h>
#include "hardware/pio.h"
#include "pio/pio_spi.h"
#include "pico/time.h"
#include "pio/linkcable_passive.h"
#include "pio/useful_qualifiers.h"
#include "pio/time_defs.h"
#include "utils.h"

#define NUM_CMP_BYTES 0x20
#define NUM_CMP_BYTES_RECV (NUM_CMP_BYTES+4)
#define PROTOCOL_VERSION_NUMBER 2

// USB Opcodes
#define OPCODE_USB_SET_ACTIVE_MODE 1 // Switch to active mode
#define OPCODE_USB_SET_PASSIVE_MODE 2 // Switch to passive mode
#define OPCODE_USB_RESET_PASSIVE_MODE_BUFFERS 3 // Reset passive mode buffers if an error happens
#define OPCODE_USB_SET_PASSIVE_MODE_DEFAULT 4 // Sets the bytes to send to the host if nothing is enqueued
#define OPCODE_USB_SET_PASSIVE_MODE_SECTION_PREAMBLE_WAITING 5 // Section Preamble mode: prepare preamble_data_to_find, data_waiting_for_preamble
#define OPCODE_USB_PASSIVE_MODE_SECTION_GET_CHECKSUM_AND_LEN 6 // Get length of the data loaded in and checksum, to ensure the transfer went well
#define OPCODE_USB_PASSIVE_MODE_SECTION_START 7 // Start section preamble mode officially
#define OPCODE_USB_PASSIVE_MODE_SECTION_GET_IS_DONE_AND_LEN 8 // Get if the section preamble mode is done, as well as the current length
#define OPCODE_USB_RETURN_TO_REGULAR_PASSIVE_MODE 9 // Return to the regular passive mode from the section preamble mode
#define OPCODE_USB_RESET_PASSIVE_MODE 10 // Reset passive mode entirely in case of an error in bits trasmitted
#define OPCODE_USB_PASSIVE_MODE_GET_SECTION_DATA 11 // Request for some section data

#define NUM_DEFAULT_BYTES_PER_TRANSFER 1
#define US_DEFAULT_PER_TRANSFER 1000

#define MAX_TRANSFER_BYTES 0x40

#define PIN_SCK 0
#define PIN_SIN 1
#define TEST_PIN 6

uint PIN_SOUT = 2;
uint SI_PIN = 3;

bool is_test_pin_grounded() {
  gpio_init(TEST_PIN);
  gpio_set_dir(TEST_PIN, GPIO_OUT);
  gpio_put(TEST_PIN, 1);  // Set the pin high

  // Read the state of the pin
  bool grounded = gpio_get(TEST_PIN) == 0;

  return grounded;
}

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED     = 1000,
  BLINK_SUSPENDED   = 2500,

  BLINK_ALWAYS_ON   = UINT32_MAX,
  BLINK_ALWAYS_OFF  = 0
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;
static uint8_t data_buf[MAX_TRANSFER_BYTES];
static uint8_t compare_bytes[NUM_CMP_BYTES] = {0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE, 0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF};
static uint8_t buf_count;
static uint8_t num_bytes_per_transfer = NUM_DEFAULT_BYTES_PER_TRANSFER;
static uint32_t us_between_transfer = US_DEFAULT_PER_TRANSFER;
static uint32_t total_transferred = 0;
static user_time_t last_active_transfer_time = 0;

// Determine which of the two modes to use...
static bool is_mode_active = true;

// Data for Passive mode
# include "io_buffer.h" // Buffers which will keep the data to send/receive.
static uint32_t passive_no_data_default = 0; // Used when there is no data to send...
static bool passive_is_scheduled_real_data = false; // Used to distinguish when data should be saved.

static uint32_t preamble_data_to_find = 0; // Used for preamble-section mode...
static uint32_t data_waiting_for_preamble = 0; // Data sent while waiting for the preamble.
static bool is_preamble_mode_being_setup = false;
static bool is_preamble_mode_requested = false;
static bool has_preamble_started = false;
static bool is_past_preamble = false;

#define URL  "tetris.gblink.io"

const tusb_desc_webusb_url_t desc_url =
{
  .bLength         = 3 + sizeof(URL) - 1,
  .bDescriptorType = 3, // WEBUSB URL type
  .bScheme         = 1, // 0: http, 1: https
  .url             = URL
};

static bool web_serial_connected = false;

//------------- prototypes -------------//
void handle_input_data(uint8_t* buf_in, uint32_t count);
void handle_output_data(void);
void led_blinking_task(void);
void cdc_task(void);
void webserial_task(void);
void TIME_SENSITIVE(link_cable_passive_handler)(void);

/*------------- MAIN -------------*/

  pio_spi_inst_t spi = {
          .pio = pio1,
          .sm = 0,
          .cpol = 1,
  };

void pio_spi_active_enable() {
    last_active_transfer_time = TIME_FUNCTION;
    pio_spi_active_activate(spi.pio, spi.sm, spi.program_offset, spi.cpol, PIN_SCK, PIN_SIN, PIN_SOUT);
}

void pio_spi_active_disable() {
    pio_sm_set_enabled(spi.pio, spi.sm, false);
    pio_sm_clear_fifos(spi.pio, spi.sm);
    pio_sm_restart(spi.pio, spi.sm);
    pio_sm_clkdiv_restart(spi.pio, spi.sm);
    pio_sm_exec(spi.pio, spi.sm, pio_encode_jmp(spi.program_offset));
}

int main(void)
{
/*
  // Fails on Stacksmashing board...
  // Code for clones...
  // Check the state of TEST_PIN
  if (is_test_pin_grounded()) {
    // GPIO 6 (TEST_PIN) is grounded, update PIN_SOUT and SI_PIN
    PIN_SOUT = 3;
    SI_PIN = 4;
  }
  else {
    // GPIO 6 (TEST_PIN) is not grounded, use default values
    PIN_SOUT = 2;
    SI_PIN = 3;
  }
*/
  //board_init();
  buf_count = 0;
  uint cpha1_prog_offs = pio_add_program(spi.pio, &spi_cpha1_active_program);
  spi.program_offset = cpha1_prog_offs;
  pio_spi_active_init_partial(spi.pio, spi.sm, spi.program_offset, 8, 4058.838/128, PIN_SCK, PIN_SIN, PIN_SOUT);

  linkcable_passive_init(link_cable_passive_handler, 1, 1, 8, PIN_SCK, PIN_SIN, PIN_SOUT);

  pio_spi_active_enable();
  //linkcable_passive_enable();
  //is_mode_active = false;

  tusb_init();

  while (1)
  {
    tud_task(); // tinyusb device task
    cdc_task();
    webserial_task();
    handle_output_data();
    led_blinking_task();
  }

  return 0;
}

int oldmain(void)
{
  board_init();

  tusb_init();

  while (1)
  {
    tud_task(); // tinyusb device task
    cdc_task();
    webserial_task();
    handle_output_data();
    led_blinking_task();
  }

  return 0;
}

void TIME_SENSITIVE(link_cable_passive_handler)(void) {
    uint32_t recieved_data = linkcable_passive_receive();

    if(passive_is_scheduled_real_data) {
      set_data_out((uint8_t*)&recieved_data, num_bytes_per_transfer, 0);
    }

    passive_is_scheduled_real_data = false;
    uint32_t data = passive_no_data_default;

    if(is_preamble_mode_being_setup) {
      // Do nothing, as the default behaviour is fine
    }
    else if(is_preamble_mode_requested) {
      if((!has_preamble_started) && (recieved_data == preamble_data_to_find))
        has_preamble_started = true;
      if((!is_past_preamble) && has_preamble_started && (recieved_data != preamble_data_to_find)) {
        is_past_preamble = true;
        // Insert the first byte
        set_data_out((uint8_t*)&recieved_data, num_bytes_per_transfer, 0);
      }

      if(!has_preamble_started)
        data = data_waiting_for_preamble;
      else if(!is_past_preamble)
        data = preamble_data_to_find;
      else if(available_data_in() > 0) {
        data = get_data_in();
        // The last byte won't be part of the section - Do not add it
        if(available_data_in() > 0)
            passive_is_scheduled_real_data = true;
      }
    }
    else if(available_data_in() > 0) {
      data = get_data_in();
      passive_is_scheduled_real_data = true;
    }

    linkcable_passive_flush();
    linkcable_passive_send(data);
}

// send characters to both CDC and WebUSB
void echo_all(uint8_t buf[], uint32_t count)
{
  // echo to web serial
  if ( web_serial_connected )
  {
    tud_vendor_write(buf, count);
    tud_vendor_flush();
  }

  // echo to cdc
  if ( tud_cdc_connected() )
  {
    for(uint32_t i=0; i<count; i++)
    {
      tud_cdc_write_char(buf[i]);
    }
    tud_cdc_write_flush();
  }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

//--------------------------------------------------------------------+
// WebUSB use vendor class
//--------------------------------------------------------------------+

// Invoked when a control transfer occurred on an interface of this class
// Driver response accordingly to the request and the transfer stage (setup/data/ack)
// return false to stall control endpoint (e.g unsupported request)
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request)
{
  // nothing to do for DATA & ACK stage
  if (stage != CONTROL_STAGE_SETUP) return true;

  switch (request->bRequest)
  {
    case VENDOR_REQUEST_WEBUSB:
      // match vendor request in BOS descriptor
      // Get landing page url
      return tud_control_xfer(rhport, request, (void*) &desc_url, desc_url.bLength);

    case VENDOR_REQUEST_MICROSOFT:
      if ( request->wIndex == 7 )
      {
        // Get Microsoft OS 2.0 compatible descriptor
        uint16_t total_len;
        memcpy(&total_len, desc_ms_os_20+8, 2);

        return tud_control_xfer(rhport, request, (void*) desc_ms_os_20, total_len);
      }else
      {
        return false;
      }
    case 0x22:
      // Webserial simulate the CDC_REQUEST_SET_CONTROL_LINE_STATE (0x22) to connect and disconnect.
      web_serial_connected = (request->wValue != 0);
      
      total_transferred = 0;
      num_bytes_per_transfer = NUM_DEFAULT_BYTES_PER_TRANSFER;
      us_between_transfer = US_DEFAULT_PER_TRANSFER;

      // Always lit LED if connected
      if ( web_serial_connected )
      {
        board_led_write(true);
        blink_interval_ms = BLINK_ALWAYS_ON;

        // tud_vendor_write_str("\r\nTinyUSB WebUSB device example\r\n");
      }else
      {
        blink_interval_ms = BLINK_MOUNTED;
      }

      // response with status OK
      return tud_control_status(rhport, request);
      break;

    default: break;
  }

  // stall unknown request
  return false;
}

// Invoked when DATA Stage of VENDOR's request is complete
bool tud_vendor_control_complete_cb(uint8_t rhport, tusb_control_request_t const * request)
{
  (void) rhport;
  (void) request;

  // nothing to do
  return true;
}

static void process_base_config_packet(uint8_t* buf_in) {
  bool has_found_non_ff = false;
  uint8_t processed = 0;

  for(int i = NUM_CMP_BYTES; i < NUM_CMP_BYTES_RECV; i++)
    if(buf_in[i] != 0xFF) {
      has_found_non_ff = true;
      break;
    }

  if(!has_found_non_ff) {
    // Make this a version return
    processed = PROTOCOL_VERSION_NUMBER;
    echo_all(&processed, 1);
    return;
  }

  us_between_transfer = (buf_in[NUM_CMP_BYTES]<<0) + (buf_in[NUM_CMP_BYTES+1]<<8) + (buf_in[NUM_CMP_BYTES+2]<<16);
  num_bytes_per_transfer = buf_in[NUM_CMP_BYTES+3];
  if(num_bytes_per_transfer > MAX_TRANSFER_BYTES)
    num_bytes_per_transfer = MAX_TRANSFER_BYTES;
  processed = 1;
  echo_all(&processed, 1);
}

static void reset_preamble_section_data() {
  is_preamble_mode_being_setup = false;
  is_preamble_mode_requested = false;
  has_preamble_started = false;
  is_past_preamble = false;
}

static bool process_special_packets(uint8_t* buf_in, uint32_t count) {
  // Is this a special packet?
  // Check if it is. If this returns true, handle it.
  uint8_t processed = 0;
  if(count >= (NUM_CMP_BYTES + 1)) {
    processed = 1;
    for(int i = 0; i < NUM_CMP_BYTES; i++)
      if(buf_in[i] != compare_bytes[i]) {
        processed = 0;
        break;
      }
  }

  if(!processed)
    return false;

  // Base message for active reconfiguration.
  if(count == NUM_CMP_BYTES_RECV) {
      process_base_config_packet(buf_in);
      return true;
  }

  size_t start_data_post_opcode = NUM_CMP_BYTES + 1;
  size_t size_post_opcode = count - start_data_post_opcode;
  uint8_t buffer_out[MAX_TRANSFER_BYTES];
  size_t length_field = 0;
  processed = PROTOCOL_VERSION_NUMBER;

  switch(buf_in[NUM_CMP_BYTES]) {
    // Switch to active mode
    case OPCODE_USB_SET_ACTIVE_MODE:
      if(size_post_opcode == 0) {
        if(!is_mode_active) {
          is_mode_active = true;
          linkcable_passive_disable();
          pio_spi_active_enable();
        }
        echo_all(&processed, 1);
        return true;
      }
      break;
    // Switch to passive mode
    case OPCODE_USB_SET_PASSIVE_MODE:
      if(size_post_opcode == 0) {
        if(is_mode_active) {
          reset_preamble_section_data();
          passive_is_scheduled_real_data = false;
          reset_data_out();
          reset_data_in();
          is_mode_active = false;
          pio_spi_active_disable();
          linkcable_passive_enable();
        }
        echo_all(&processed, 1);
        return true;
      }
    // Reset passive mode buffers if an error happens
    case OPCODE_USB_RESET_PASSIVE_MODE_BUFFERS:
      if(size_post_opcode == 0) {
        reset_data_out();
        reset_data_in();
        echo_all(&processed, 1);
        return true;
      }
      break;
    // Sets the bytes to send to the host if nothing is enqueued
    case OPCODE_USB_SET_PASSIVE_MODE_DEFAULT:
      if(size_post_opcode == sizeof(uint32_t)) {
        passive_no_data_default = read_u32_le(buf_in, start_data_post_opcode);
        write_u32_le(buffer_out, 0, passive_no_data_default);
        echo_all(buffer_out, sizeof(uint32_t));
        return true;
      }
      break;
    // Section Preamble mode: prepare preamble_data_to_find, data_waiting_for_preamble
    //https://github.com/pret/pokecrystal/blob/619f3703486b3b40f69d31b1911b58aa0a721a6e/home/serial.asm#L91
    case OPCODE_USB_SET_PASSIVE_MODE_SECTION_PREAMBLE_WAITING:
      if(size_post_opcode == (sizeof(uint32_t) + sizeof(uint32_t))) {
        passive_is_scheduled_real_data = false;
        reset_data_out();
        reset_data_in();
        reset_preamble_section_data();
        is_preamble_mode_being_setup = true;
        is_preamble_mode_requested = true;
        preamble_data_to_find = read_u32_le(buf_in, start_data_post_opcode);
        data_waiting_for_preamble = read_u32_le(buf_in, start_data_post_opcode + sizeof(uint32_t));
        write_u32_le(buffer_out, 0, preamble_data_to_find);
        write_u32_le(buffer_out, sizeof(uint32_t), data_waiting_for_preamble);
        echo_all(buffer_out, sizeof(uint32_t) + sizeof(uint32_t));
        return true;
      }
      break;
    // Get length of the data loaded in and checksum, to ensure the transfer went well
    case OPCODE_USB_PASSIVE_MODE_SECTION_GET_CHECKSUM_AND_LEN:
      if(size_post_opcode == 0) {
        write_u32_le(buffer_out, 0, get_data_in_checksum());
        write_u32_le(buffer_out, sizeof(uint32_t), available_data_in());
        echo_all(buffer_out, sizeof(uint32_t) + sizeof(uint32_t));
        return true;
      }
      break;
    // Start section preamble mode officially
    case OPCODE_USB_PASSIVE_MODE_SECTION_START:
      if(size_post_opcode == 0) {
        is_preamble_mode_being_setup = false;
        echo_all(&processed, 1);
        return true;
      }
      break;
    // Get if the section preamble mode is done, as well as the current length
    case OPCODE_USB_PASSIVE_MODE_SECTION_GET_IS_DONE_AND_LEN:
      if(size_post_opcode == 0) {
        buffer_out[0] = 0;
        size_t curr_to_transfer = available_data_in();
        if((curr_to_transfer == 0) && (!passive_is_scheduled_real_data))
          buffer_out[0] = 1;
        write_u32_le(buffer_out, 1, available_data_out());
        write_u32_le(buffer_out, 1 + sizeof(uint32_t), curr_to_transfer);
        echo_all(buffer_out, 1 + sizeof(uint32_t) + sizeof(uint32_t));
        return true;
      }
      break;
    // Return to the regular passive mode from the section preamble mode
    case OPCODE_USB_RETURN_TO_REGULAR_PASSIVE_MODE:
      if(size_post_opcode == 0) {
        passive_is_scheduled_real_data = false;
        reset_data_out();
        reset_data_in();
        reset_preamble_section_data();
        echo_all(&processed, 1);
        return true;
      }
      break;
    // Reset passive mode entirely in case of an error in bits trasmitted
    case OPCODE_USB_RESET_PASSIVE_MODE:
      if(size_post_opcode == 0) {
        if(!is_mode_active) {
          linkcable_passive_disable();
          passive_is_scheduled_real_data = false;
          reset_data_out();
          reset_data_in();
          reset_preamble_section_data();
          linkcable_passive_enable();
        }
        echo_all(&processed, 1);
        return true;
      }
      break;
    // Host request for some section data
    case OPCODE_USB_PASSIVE_MODE_GET_SECTION_DATA:
      if(size_post_opcode == sizeof(uint32_t)) {
        if(is_preamble_mode_requested) {
          // Validate length request and insert it in data returned...
          length_field = read_u32_le(buf_in, start_data_post_opcode);
          if(length_field > available_data_out())
            length_field = available_data_out();
          if(length_field > (MAX_TRANSFER_BYTES - sizeof(uint32_t)))
            length_field = MAX_TRANSFER_BYTES - sizeof(uint32_t);

          // Extract the data from the buffer and put it in...
          bool success = false;
          uint8_t total_processed = 0;
          for(size_t i = 0; i < length_field; i++) {
            uint8_t data = get_data_out(&success);
            if(success)
              buffer_out[(total_processed++) + sizeof(uint32_t)] = data;
          }
          length_field = total_processed;
        }

        write_u32_le(buffer_out, 0, length_field);
        echo_all(buffer_out, length_field + sizeof(uint32_t));
        return true;
      }
      break;
    default:
      break;
  }

  return false;
}

void handle_input_data(uint8_t* buf_in, uint32_t count) {
  for(int i = count; i < (MAX_TRANSFER_BYTES*2); i++)
    buf_in[i] = 0;

  bool has_been_processed = process_special_packets(buf_in, count);

  if(!has_been_processed) {
    if(is_mode_active) {
      // pprintf("Sending: %02x", buf[0]);
      uint8_t total_processed = 0;
      uint8_t buf_out[MAX_TRANSFER_BYTES*2];
      while(total_processed < count) {
        uint8_t transferable = num_bytes_per_transfer;
        //if(count-total_processed < transferable)
          //transferable = count-total_processed;
        user_time_t curr_time = TIME_FUNCTION;
        user_time_comparison_t remaining_time = (((user_time_comparison_t)last_active_transfer_time) + MUSEC(us_between_transfer)) - ((user_time_comparison_t)curr_time);
        if(remaining_time > 0)
          busy_wait_us(remaining_time);
        pio_spi_write8_read8_blocking(&spi, buf_in + total_processed, buf_out + total_processed, transferable);
        total_transferred += transferable;
        total_processed += transferable;
        last_active_transfer_time = TIME_FUNCTION;
      }
      echo_all(buf_out, total_processed);
      //echo_all(&availables, 1);
    }
    else
      set_data_in(buf_in, count);
  }
}

void handle_output_data() {
  if(is_mode_active)
    return;

  // Periodically check for issues
  linkcable_passive_check_for_transmission_error();

  size_t possible = available_data_out();
  if(possible == 0)
    return;
  // Handle section transfers in a special way
  if(is_preamble_mode_requested)
    return;
  if(possible > MAX_TRANSFER_BYTES)
    possible = MAX_TRANSFER_BYTES;

  uint8_t total_processed = 0;
  uint8_t buf_out[MAX_TRANSFER_BYTES*2];
  bool success = false;

  for(size_t i = 0; i < possible; i++) {
    uint8_t data = get_data_out(&success);
    if(success)
      buf_out[total_processed++] = data;
  }

  echo_all(buf_out, total_processed);
}

void webserial_task(void)
{
  if ( web_serial_connected ) {
    if ( tud_vendor_available() ) {
      uint8_t buf_in[MAX_TRANSFER_BYTES*2];
      uint32_t count = tud_vendor_read(buf_in, sizeof(buf_in));
      handle_input_data(buf_in, count);
    }
  }
}


//--------------------------------------------------------------------+
// USB CDC
//--------------------------------------------------------------------+
void cdc_task(void)
{
  if ( tud_cdc_connected() ) {
    // connected and there are data available
    if ( tud_cdc_available() ) {
      uint8_t buf_in[MAX_TRANSFER_BYTES*2];
      uint32_t count = tud_cdc_read((uint8_t*)buf_in, sizeof(buf_in));
      handle_input_data(buf_in, count);
    }
  }
}

// Invoked when cdc when line state changed e.g connected/disconnected
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
  (void) itf;

  // connected
  if ( dtr && rts )
  {
    // print initial message when connected
    // tud_cdc_write_str("\r\nTinyUSB WebUSB device example\r\n");
  }
}

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf)
{
  (void) itf;
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void)
{
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // Blink every interval ms
  if ( board_millis() - start_ms < blink_interval_ms) return; // not enough time
  start_ms += blink_interval_ms;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
}
