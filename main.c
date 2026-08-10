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
#include "hardware/clocks.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"

// --- WS2812 NeoPixel PIO Driver ---
static const uint16_t ws2812_program_instructions[] = {
    0x6221, 0x1123, 0x1400, 0xa442
};

static const struct pio_program ws2812_program = {
    .instructions = ws2812_program_instructions,
    .length = 4,
    .origin = -1,
};

static inline void ws2812_program_init(PIO pio, uint sm, uint offset, uint pin, float freq) {
    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_sideset(&c, 1, false, false);
    sm_config_set_sideset_pins(&c, pin);
    sm_config_set_out_shift(&c, false, true, 24);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_wrap(&c, offset, offset + 3);
    sm_config_set_clkdiv(&c, clock_get_hz(clk_sys) / (freq * 8.0f));

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

void set_neopixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

#define NUM_CMP_BYTES 0x20
#define NUM_CMP_BYTES_RECV (NUM_CMP_BYTES+4)

#define NUM_DEFAULT_BYTES_PER_TRANSFER 1
#define US_DEFAULT_PER_TRANSFER 1000

#define MAX_TRANSFER_BYTES 0x40

// Printer mode constants
#define PRINTER_MODE_MAGIC_LEN 36
static uint8_t printer_mode_magic[PRINTER_MODE_MAGIC_LEN] = {
    0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE,
    0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE,
    0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
    0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
    'P', 'R', 'N', 'T'  // "PRNT" to indicate printer mode
};
static bool printer_mode = false;

// Voltage switch magic packets (36 bytes each, same length as printer mode)
#define VSWITCH_MAGIC_LEN 36
static uint8_t vswitch_3v3_magic[VSWITCH_MAGIC_LEN] = {
    0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE,
    0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE,
    0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
    0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
    'V', '3', 'V', '3'  // "V3V3" to indicate 3.3V mode
};
static uint8_t vswitch_5v_magic[VSWITCH_MAGIC_LEN] = {
    0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE,
    0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE,
    0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
    0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
    'V', '5', 'V', '0'  // "V5V0" to indicate 5V mode
};

// LED color magic packet: 36-byte header + R, G, B, on/off = 40 bytes total
#define LED_MAGIC_LEN 40
#define LED_MAGIC_HEADER_LEN 36
static uint8_t led_magic_header[LED_MAGIC_HEADER_LEN] = {
    0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE,
    0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE,
    0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
    0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
    'L', 'E', 'D', 'S'  // "LEDS" to indicate LED color command
};

// WebUSB landing-page toggle: 36-byte header + 1 value byte = 37 bytes total.
// value 0x00 = disable, 0x01 = enable, 0xFF = query only. Always responds with
// the current state (0/1). Persisted in flash; applies on the next reconnect.
#define LANDING_MAGIC_LEN 37
#define LANDING_MAGIC_HEADER_LEN 36
static uint8_t landing_magic_header[LANDING_MAGIC_HEADER_LEN] = {
    0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE,
    0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE,
    0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
    0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
    'L', 'A', 'N', 'D'  // "LAND" to indicate landing-page toggle/query
};

// Per-mode LED colour commands. Both use length 40 (like the live "LEDS"
// command) and are distinguished by the 4-char tag at offset 32 — length 40
// avoids the 36-byte timing command, which only matches the 32-byte prefix.
//   LEDM (set):  32-byte prefix + "LEDM" + [slot, r, g, b]
//   LEDG (query): 32-byte prefix + "LEDG" + 4 ignored bytes  -> replies [count, rgb...]
#define LED_CMD_MAGIC_LEN 40
#define LED_CMD_HEADER_LEN 36
static uint8_t led_mode_magic_header[LED_CMD_HEADER_LEN] = {
    0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE,
    0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE,
    0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
    0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
    'L', 'E', 'D', 'M'
};
static uint8_t led_get_magic_header[LED_CMD_HEADER_LEN] = {
    0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE,
    0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE,
    0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
    0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
    'L', 'E', 'D', 'G'
};
static uint8_t led_reset_magic_header[LED_CMD_HEADER_LEN] = {
    0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE,
    0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE,
    0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
    0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
    'L', 'E', 'D', 'R'
};
// Warm-reboot the board into the app (not the bootloader) so persisted settings
// apply now: 32-byte prefix + "BOOT". Same length (40) as the LED family.
static uint8_t reboot_magic_header[LED_CMD_HEADER_LEN] = {
    0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE,
    0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE, 0xCA, 0xFE,
    0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
    0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
    'B', 'O', 'O', 'T'
};

// Persistent settings (last 4 KB flash sector, well past the ~55 KB app): the
// WebUSB landing-page flag and per-mode LED colours. One versioned struct;
// erased/old flash falls back to defaults via the magic/version check.
#define SETTINGS_FLASH_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define SETTINGS_MAGIC 0x5A
#define SETTINGS_VERSION 4  // bumped to re-default the LED colours
#define RECFG_LED_SLOTS 3  // 0 = connected, 1 = active/link, 2 = printer

typedef struct {
    uint8_t magic;
    uint8_t version;
    uint8_t landing;                  // 0 = disabled, else enabled
    uint8_t led[RECFG_LED_SLOTS][3];  // logical RGB
} persist_settings_t;

// Real-colour defaults at a uniform 50% brightness (128/255).
static const uint8_t recfg_default_colors[RECFG_LED_SLOTS][3] = {
    { 0,   128, 0   }, // connected — green
    { 0,   0,   128 }, // active / link — blue
    { 128, 0,   128 }, // printer — purple
};

static void load_settings(persist_settings_t *s) {
    memcpy(s, (const void *)(XIP_BASE + SETTINGS_FLASH_OFFSET), sizeof(*s));
    if (s->magic != SETTINGS_MAGIC || s->version != SETTINGS_VERSION) {
        s->magic = SETTINGS_MAGIC;
        s->version = SETTINGS_VERSION;
        s->landing = 1;
        memcpy(s->led, recfg_default_colors, sizeof(s->led));
    }
}

static void save_settings(const persist_settings_t *s) {
    uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, sizeof(page));
    memcpy(page, s, sizeof(*s));
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(SETTINGS_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(SETTINGS_FLASH_OFFSET, page, FLASH_PAGE_SIZE);
    restore_interrupts(ints);
}

bool landing_page_enabled(void) {
    persist_settings_t s; load_settings(&s);
    return s.landing != 0;
}

void set_landing_page_enabled(bool enabled) {
    persist_settings_t s; load_settings(&s);
    s.landing = enabled ? 1 : 0;
    save_settings(&s);
}

static void get_led_color(uint8_t slot, uint8_t out[3]) {
    if (slot >= RECFG_LED_SLOTS) { out[0] = out[1] = out[2] = 0; return; }
    persist_settings_t s; load_settings(&s);
    out[0] = s.led[slot][0]; out[1] = s.led[slot][1]; out[2] = s.led[slot][2];
}

static void set_led_color(uint8_t slot, uint8_t r, uint8_t g, uint8_t b) {
    if (slot >= RECFG_LED_SLOTS) return;
    persist_settings_t s; load_settings(&s);
    s.led[slot][0] = r; s.led[slot][1] = g; s.led[slot][2] = b;
    save_settings(&s);
}

static void reset_led_colors(void) {
    persist_settings_t s; load_settings(&s);
    memcpy(s.led, recfg_default_colors, sizeof(s.led));
    save_settings(&s);
}

// Apply a slot's stored colour now. set_neopixel wants GRB.
static void apply_led_for_slot(uint8_t slot) {
    uint8_t c[3]; get_led_color(slot, c);
    set_neopixel(((uint32_t)c[1] << 16) | ((uint32_t)c[0] << 8) | c[2]);
}

#define PIN_SCK 0
#define PIN_SIN 1
#define TEST_PIN 6
#define WS2812_PIN 16

// Voltage switch GPIO pins
#define VSWITCH_3V3_PIN 11
#define VSWITCH_5V_PIN  12

uint PIN_SOUT = 2;
uint SD_PIN = 3;

bool is_test_pin_grounded() {
  gpio_init(TEST_PIN);
  gpio_set_dir(TEST_PIN, GPIO_OUT);
  gpio_put(TEST_PIN, 1);  // Set the pin high

  // Read the state of the pin
  bool grounded = gpio_get(TEST_PIN) == 0;

  return grounded;
}

// Voltage switch control: set both HIGH first (safe off), then pull one LOW
void vswitch_set_3v3(void) {
  gpio_put(VSWITCH_3V3_PIN, 1);
  gpio_put(VSWITCH_5V_PIN, 1);
  busy_wait_us(100);
  gpio_put(VSWITCH_3V3_PIN, 0);
}

void vswitch_set_5v(void) {
  gpio_put(VSWITCH_3V3_PIN, 1);
  gpio_put(VSWITCH_5V_PIN, 1);
  busy_wait_us(100);
  gpio_put(VSWITCH_5V_PIN, 0);
}

void vswitch_init(void) {
  gpio_init(VSWITCH_3V3_PIN);
  gpio_init(VSWITCH_5V_PIN);
  gpio_set_dir(VSWITCH_3V3_PIN, GPIO_OUT);
  gpio_set_dir(VSWITCH_5V_PIN, GPIO_OUT);
  // Start both HIGH (safe), then enable 3.3V as default
  gpio_put(VSWITCH_3V3_PIN, 1);
  gpio_put(VSWITCH_5V_PIN, 1);
  busy_wait_us(100);
  gpio_put(VSWITCH_3V3_PIN, 0); // 3.3V mode (default)
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

#define URL  "launcher.gblink.io"

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
void data_transfer_task(void);
void led_blinking_task(void);
void cdc_task(void);
void webserial_task(void);
void printer_mode_loop(void);

/*------------- MAIN -------------*/

  pio_spi_inst_t spi = {
          .pio = pio1,
          .sm = 0
  };


int main(void)
{
  // Check the state of TEST_PIN
  if (is_test_pin_grounded()) {
    // GPIO 6 (TEST_PIN) is grounded, update PIN_SOUT and SD_PIN
    PIN_SOUT = 3;
    SD_PIN = 4;
  }
  else {
    // GPIO 6 (TEST_PIN) is not grounded, use default values
    PIN_SOUT = 2;
    SD_PIN = 3;
  }

  vswitch_init();

  board_init();
  uint offset = pio_add_program(pio0, &ws2812_program);
  ws2812_program_init(pio0, 0, offset, WS2812_PIN, 800000);
  buf_count = 0;
  uint cpha1_prog_offs = pio_add_program(spi.pio, &spi_cpha1_program);
  pio_spi_init(spi.pio, spi.sm, cpha1_prog_offs, 8, 4058.838/128, 1, 1, PIN_SCK, PIN_SOUT, PIN_SIN);

  tusb_init();

  while (1)
  {
    tud_task(); // tinyusb device task
    data_transfer_task();
    cdc_task();
    webserial_task();
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
    led_blinking_task();
  }

  return 0;
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
      // WebSerial connect/disconnect
      web_serial_connected = (request->wValue != 0);
      total_transferred = 0;
      num_bytes_per_transfer = NUM_DEFAULT_BYTES_PER_TRANSFER;
      us_between_transfer = US_DEFAULT_PER_TRANSFER;

      if (web_serial_connected) {
        apply_led_for_slot(1); // active / link
        blink_interval_ms = BLINK_ALWAYS_ON;
      } else {
        apply_led_for_slot(0); // connected
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

void data_transfer_task(void) {
    //if(buf_count) {
        //uint8_t buf_out[MAX_TRANSFER_BYTES];
    //    for(int i = 0; i < (buf_count+3) >> 2; i++) {
    //        pio_spi_write8_blocking(&spi, data_buf+(4*i), 4);
    //        busy_wait_us(36);
    //    }
        //pio_spi_write8_read8_blocking(&spi, data_buf, buf_out, buf_count);
        //echo_all(buf_out, buf_count);
    //    buf_count = 0;
    //}
}

void handle_input_data(uint8_t* buf_in, uint32_t count) {
  for(int i = count; i < (MAX_TRANSFER_BYTES*2); i++)
    buf_in[i] = 0;
  uint8_t processed = 0;
  // Check for printer mode magic sequence
  if(count == PRINTER_MODE_MAGIC_LEN) {
    uint8_t is_printer_mode = 1;
    for(int i = 0; i < PRINTER_MODE_MAGIC_LEN; i++) {
      if(buf_in[i] != printer_mode_magic[i]) {
        is_printer_mode = 0;
        break;
      }
    }
    if(is_printer_mode) {
      // Acknowledge and enter printer mode
      uint8_t ack = 0x50; // 'P' for printer
      echo_all(&ack, 1);
      printer_mode = true;
      apply_led_for_slot(2); // printer
      
      // Disable PIO SPI and reconfigure pins for GPIO
      pio_sm_set_enabled(spi.pio, spi.sm, false);
      
      // Run printer loop (this blocks until disconnected)
      printer_mode_loop();
      
      // Re-enable PIO SPI when exiting printer mode
      pio_sm_set_enabled(spi.pio, spi.sm, true);
      printer_mode = false;
      apply_led_for_slot(1); // active / link
      processed = 1;
    }
  }
  
  // Check for voltage switch magic sequences
  if(!processed && count == VSWITCH_MAGIC_LEN) {
    uint8_t is_3v3 = 1, is_5v = 1;
    for(int i = 0; i < VSWITCH_MAGIC_LEN; i++) {
      if(buf_in[i] != vswitch_3v3_magic[i]) is_3v3 = 0;
      if(buf_in[i] != vswitch_5v_magic[i]) is_5v = 0;
      if(!is_3v3 && !is_5v) break;
    }
    if(is_3v3) {
      vswitch_set_3v3();
      uint8_t ack = 0x33; // '3' for 3.3V
      echo_all(&ack, 1);
      processed = 1;
    } else if(is_5v) {
      vswitch_set_5v();
      uint8_t ack = 0x35; // '5' for 5V
      echo_all(&ack, 1);
      processed = 1;
    }
  }

  // Check for LED color magic sequence
  if(!processed && count == LED_MAGIC_LEN) {
    uint8_t is_led = 1;
    for(int i = 0; i < LED_MAGIC_HEADER_LEN; i++) {
      if(buf_in[i] != led_magic_header[i]) {
        is_led = 0;
        break;
      }
    }
    if(is_led) {
      uint8_t r = buf_in[LED_MAGIC_HEADER_LEN];
      uint8_t g = buf_in[LED_MAGIC_HEADER_LEN + 1];
      uint8_t b = buf_in[LED_MAGIC_HEADER_LEN + 2];
      uint8_t on = buf_in[LED_MAGIC_HEADER_LEN + 3];
      if(on) {
        // NeoPixel expects GRB format
        set_neopixel(((uint32_t)g << 16) | ((uint32_t)r << 8) | b);
      } else {
        set_neopixel(0x000000);
      }
      // Disable blink task from overriding the color
      blink_interval_ms = BLINK_ALWAYS_ON;
      uint8_t ack = 0x4C; // 'L' for LED
      echo_all(&ack, 1);
      processed = 1;
    }
  }

  // Set a per-mode LED colour (persisted): prefix + "LEDM" + [slot, r, g, b]
  if(!processed && count == LED_CMD_MAGIC_LEN) {
    uint8_t is_ledm = 1;
    for(int i = 0; i < LED_CMD_HEADER_LEN; i++) {
      if(buf_in[i] != led_mode_magic_header[i]) { is_ledm = 0; break; }
    }
    if(is_ledm) {
      set_led_color(buf_in[LED_CMD_HEADER_LEN], buf_in[LED_CMD_HEADER_LEN + 1],
                    buf_in[LED_CMD_HEADER_LEN + 2], buf_in[LED_CMD_HEADER_LEN + 3]);
      uint8_t ack = 0x4D; // 'M'
      echo_all(&ack, 1);
      processed = 1;
    }
  }

  // Query per-mode LED colours: prefix + "LEDG" -> [count, r0,g0,b0, ...]
  if(!processed && count == LED_CMD_MAGIC_LEN) {
    uint8_t is_ledg = 1;
    for(int i = 0; i < LED_CMD_HEADER_LEN; i++) {
      if(buf_in[i] != led_get_magic_header[i]) { is_ledg = 0; break; }
    }
    if(is_ledg) {
      uint8_t resp[1 + RECFG_LED_SLOTS * 3];
      resp[0] = RECFG_LED_SLOTS;
      for(uint8_t slot = 0; slot < RECFG_LED_SLOTS; slot++) {
        get_led_color(slot, &resp[1 + slot * 3]);
      }
      echo_all(resp, sizeof(resp));
      processed = 1;
    }
  }

  // Reset per-mode LED colours to defaults: prefix + "LEDR"
  if(!processed && count == LED_CMD_MAGIC_LEN) {
    uint8_t is_ledr = 1;
    for(int i = 0; i < LED_CMD_HEADER_LEN; i++) {
      if(buf_in[i] != led_reset_magic_header[i]) { is_ledr = 0; break; }
    }
    if(is_ledr) {
      reset_led_colors();
      uint8_t ack = 0x52; // 'R'
      echo_all(&ack, 1);
      processed = 1;
    }
  }

  // Warm-reboot into the app so saved settings apply now: prefix + "BOOT"
  if(!processed && count == LED_CMD_MAGIC_LEN) {
    uint8_t is_boot = 1;
    for(int i = 0; i < LED_CMD_HEADER_LEN; i++) {
      if(buf_in[i] != reboot_magic_header[i]) { is_boot = 0; break; }
    }
    if(is_boot) {
      uint8_t ack = 0x42; // 'B'
      echo_all(&ack, 1);
      tud_vendor_flush();
      // Let the ack drain to the host, then reboot. watchdog_reboot(0,0,delay)
      // performs a normal boot back into the application.
      busy_wait_ms(20);
      watchdog_reboot(0, 0, 0); // does not return
    }
  }

  // Check for WebUSB landing-page toggle/query magic sequence
  if(!processed && count == LANDING_MAGIC_LEN) {
    uint8_t is_land = 1;
    for(int i = 0; i < LANDING_MAGIC_HEADER_LEN; i++) {
      if(buf_in[i] != landing_magic_header[i]) {
        is_land = 0;
        break;
      }
    }
    if(is_land) {
      uint8_t value = buf_in[LANDING_MAGIC_HEADER_LEN];
      if(value != 0xFF) {  // 0xFF = query only, don't change
        set_landing_page_enabled(value != 0);
      }
      uint8_t ack = landing_page_enabled() ? 1 : 0;
      echo_all(&ack, 1);
      processed = 1;
    }
  }

  // Check for timing config magic sequence
  if(!processed && count == NUM_CMP_BYTES_RECV) {
    uint8_t failed = 0;
    for(int i = 0; i < NUM_CMP_BYTES; i++)
      if(buf_in[i] != compare_bytes[i]) {
        failed = 1;
        break;
      }
    if(!failed) {
      us_between_transfer = (buf_in[NUM_CMP_BYTES]<<0) + (buf_in[NUM_CMP_BYTES+1]<<8) + (buf_in[NUM_CMP_BYTES+2]<<16);
      num_bytes_per_transfer = buf_in[NUM_CMP_BYTES+3];
      if(num_bytes_per_transfer > MAX_TRANSFER_BYTES)
        num_bytes_per_transfer = MAX_TRANSFER_BYTES;
      processed = 1;
      echo_all(&processed, 1);
    }
  }
  if(!processed) {
    // pprintf("Sending: %02x", buf[0]);
    uint8_t total_processed = 0;
    uint8_t buf_out[MAX_TRANSFER_BYTES*2];
    while(total_processed < count) {
      uint8_t transferable = num_bytes_per_transfer;
      //if(count-total_processed < transferable)
        //transferable = count-total_processed;
      pio_spi_write8_read8_blocking(&spi, buf_in + total_processed, buf_out + total_processed, transferable);
      total_transferred += transferable;
      total_processed += transferable;
      busy_wait_us(us_between_transfer);
    }
    echo_all(buf_out, total_processed);
    //echo_all(&availables, 1);
  }
}

void webserial_task(void)
{
  if ( web_serial_connected )
    if ( tud_vendor_available() ) {
      uint8_t buf_in[MAX_TRANSFER_BYTES*2];
      uint32_t count = tud_vendor_read(buf_in, sizeof(buf_in));
      handle_input_data(buf_in, count);
    }
}


//--------------------------------------------------------------------+
// USB CDC
//--------------------------------------------------------------------+
void cdc_task(void)
{
  if ( tud_cdc_connected() )
    // connected and there are data available
    if ( tud_cdc_available() ) {
      uint8_t buf_in[MAX_TRANSFER_BYTES*2];
      uint32_t count = tud_cdc_read((uint8_t*)buf_in, sizeof(buf_in));
      handle_input_data(buf_in, count);
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

  if (board_millis() - start_ms < blink_interval_ms) return;
  start_ms += blink_interval_ms;

  led_state = !led_state;

  if (blink_interval_ms == BLINK_ALWAYS_ON) {
    // Solid color - handled when state changes, don't toggle
    return;
  }

  if (led_state) {
    if (blink_interval_ms == BLINK_MOUNTED)
      apply_led_for_slot(0); // connected
    else if (blink_interval_ms == BLINK_NOT_MOUNTED)
      set_neopixel(0x000500); // Red
  } else {
    set_neopixel(0x000000); // Off
  }
}

//--------------------------------------------------------------------+
// PRINTER MODE - GPIO bit-bang SPI slave with protocol handling
//--------------------------------------------------------------------+

// Printer protocol states
enum printer_state {
  GB_WAIT_FOR_SYNC_1,
  GB_WAIT_FOR_SYNC_2,
  GB_COMMAND,
  GB_COMPRESSION_INDICATOR,
  GB_LEN_LOWER,
  GB_LEN_HIGHER,
  GB_DATA,
  GB_CHECKSUM_1,
  GB_CHECKSUM_2,
  GB_SEND_DEVICE_ID,
  GB_SEND_STATUS
};

void printer_mode_loop(void) {
  // Reconfigure pins for GPIO bit-bang mode (SPI slave)
  gpio_init(PIN_SCK);
  gpio_init(PIN_SIN);
  gpio_init(PIN_SOUT);
  
  gpio_set_dir(PIN_SCK, GPIO_IN);
  gpio_set_dir(PIN_SIN, GPIO_IN);
  gpio_set_dir(PIN_SOUT, GPIO_OUT);
  gpio_put(PIN_SOUT, 0);
  
  // Bit-level variables
  uint8_t received_data = 0;
  uint8_t received_bits = 0;
  uint8_t send_data = 0x00;
  bool bit_synced = false;
  
  // Protocol state machine
  enum printer_state state = GB_WAIT_FOR_SYNC_1;
  uint8_t command = 0;
  uint16_t length = 0;
  uint16_t data_count = 0;
  uint8_t printer_status = 0x00;  // 0x00 = OK, ready
  
  // Timeout for disconnection detection
  uint32_t idle_count = 0;
  const uint32_t IDLE_TIMEOUT = 10000000;
  const uint32_t PRINT_ABORT_TIMEOUT = 2000000;  // ~200ms for mid-print cancellation detection
  
  // Send start marker to browser
  uint8_t start_marker = 0xFF;
  echo_all(&start_marker, 1);
  tud_vendor_flush();
  
  while (printer_mode && web_serial_connected) {
    // Wait for clock to go low (with timeout)
    // Use shorter timeout when mid-packet (print abort detection)
    idle_count = 0;
    uint32_t timeout = (state != GB_WAIT_FOR_SYNC_1 && state != GB_WAIT_FOR_SYNC_2) 
                       ? PRINT_ABORT_TIMEOUT : IDLE_TIMEOUT;
    
    while (gpio_get(PIN_SCK)) {
      idle_count++;
      if (idle_count > timeout) {
        tud_task();
        if (!web_serial_connected) {
          goto exit_printer_mode;
        }
        
        // If we timed out mid-packet, the print was aborted
        if (state != GB_WAIT_FOR_SYNC_1 && state != GB_WAIT_FOR_SYNC_2) {
          // Reset state machine
          state = GB_WAIT_FOR_SYNC_1;
          bit_synced = false;
          received_data = 0;
          received_bits = 0;
          send_data = 0x00;
          
          // Send "ABORTPRINT" to browser to signal print abort
          // ASCII string is impossible to occur in Game Boy tile data
          const char* abort_marker = "ABORTPRINT";
          echo_all((uint8_t*)abort_marker, 10);
          tud_vendor_flush();
          
          // Break out of clock wait loop to start fresh
          break;
        }
        
        idle_count = 0;
      }
    }
    
    // Clock is LOW - output our bit (LSB first)
    gpio_put(PIN_SOUT, send_data & 0x1);
    send_data = send_data >> 1;
    
    // Wait for clock to go high
    while (!gpio_get(PIN_SCK)) {}
    
    // Clock is HIGH - sample input bit (MSB first)
    received_data = (received_data << 1) | (gpio_get(PIN_SIN) & 0x1);
    
    // Bit sync detection - look for 0x88 pattern
    if (!bit_synced) {
      if (received_data != 0x88) {
        continue;
      } else {
        received_bits = 8;
        bit_synced = true;
      }
    } else {
      received_bits++;
    }
    
    // Check if we have a complete byte
    if (received_bits != 8) {
      continue;
    }
    
    // We have a complete byte - process it
    uint8_t byte = received_data;
    received_data = 0;
    received_bits = 0;
    
    // Protocol state machine - handle byte and set response for NEXT byte
    switch (state) {
      case GB_WAIT_FOR_SYNC_1:
        if (byte == 0x88) {
          state = GB_WAIT_FOR_SYNC_2;
        }
        send_data = 0x00;
        break;
        
      case GB_WAIT_FOR_SYNC_2:
        if (byte == 0x33) {
          state = GB_COMMAND;
        } else {
          state = GB_WAIT_FOR_SYNC_1;
          bit_synced = false;
        }
        send_data = 0x00;
        break;
        
      case GB_COMMAND:
        command = byte;
        state = GB_COMPRESSION_INDICATOR;
        send_data = 0x00;
        
        // Send command to browser for logging
        echo_all(&byte, 1);
        break;
        
      case GB_COMPRESSION_INDICATOR:
        state = GB_LEN_LOWER;
        send_data = 0x00;
        // Send compression byte to browser
        echo_all(&byte, 1);
        break;
        
      case GB_LEN_LOWER:
        length = byte;
        state = GB_LEN_HIGHER;
        send_data = 0x00;
        break;
        
      case GB_LEN_HIGHER:
        length |= ((uint16_t)byte << 8);
        data_count = 0;
        
        if (length > 0) {
          state = GB_DATA;
        } else {
          state = GB_CHECKSUM_1;
        }
        send_data = 0x00;
        
        // Send length to browser (2 bytes, little endian)
        uint8_t len_bytes[2] = { length & 0xFF, (length >> 8) & 0xFF };
        echo_all(len_bytes, 2);
        break;
        
      case GB_DATA:
        data_count++;
        
        // Send data byte to browser
        echo_all(&byte, 1);
        
        if (data_count >= length) {
          state = GB_CHECKSUM_1;
        }
        send_data = 0x00;
        break;
        
      case GB_CHECKSUM_1:
        state = GB_CHECKSUM_2;
        send_data = 0x00;
        break;
        
      case GB_CHECKSUM_2:
        state = GB_SEND_DEVICE_ID;
        // Set device ID response for next exchange
        send_data = 0x81;  // Printer device ID
        break;
        
      case GB_SEND_DEVICE_ID:
        state = GB_SEND_STATUS;
        // Set status response for next exchange
        send_data = printer_status;  // 0x00 = OK
        break;
        
      case GB_SEND_STATUS:
        state = GB_WAIT_FOR_SYNC_1;
        bit_synced = false;
        send_data = 0x00;
        
        // Reset for print command
        if (command == 0x02) {  // PRINT command
          // Send print marker to browser
          uint8_t print_marker = 0xFE;
          echo_all(&print_marker, 1);
        }
        break;
    }
    
    // Flush USB periodically
    tud_task();
    tud_vendor_flush();
  }

exit_printer_mode:
  // Re-initialize GPIO for PIO SPI mode
  pio_gpio_init(spi.pio, PIN_SOUT);
  pio_gpio_init(spi.pio, PIN_SIN);
  pio_gpio_init(spi.pio, PIN_SCK);
}

