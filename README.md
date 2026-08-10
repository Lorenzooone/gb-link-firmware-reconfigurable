# GB-Link USB  -  RP2040 Game Boy Link Adapter Firmware

This firmware turns an RP2040 into a USB-to-Game Boy Link Cable adapter. It supports WebUSB for browser-based communication and a standard CDC Serial interface.

## Hardware Required
* **Controler board:** Pi-Pico or Waveshare RP2040-Zero (For LED functions)
* **Level shifter/connector board:** This one for standard Pi-Pico https://github.com/agtbaskara/game-boy-pico-link-board or this one for RP2040-Zero https://github.com/weimanc/game-boy-zero-link-board

## Wiring Guide
The firmware uses the RP2040 PIO (Programmable I/O) to communicate with the Game Boy. Connect the Link Cable wires to the RP2040 header pins as follows:

| Game Boy Signal | RP2040 Pin |
| :--- | :--- |
| **SCK** (Clock) | **GP0** |
| **SIN** (Data In) | **GP1** |
| **SOUT** (Data Out)| **GP2** |
| **SD** (Chip Select)| **GP3** |
| **GND** | **Ground** |


## LED Status Indicators (only for RP2040Zero with onboard WS2812)
The onboard RGB LED (GP16) indicates the current connection status:

| Color | Status | Description |
| :--- | :--- | :--- |
| **Red** | **Disconnected** | Device powered, but USB data not enumerated. |
| **Green** | **Mounted** | USB connected successfully to the host computer. |
| **Blue** | **Active** | WebUSB session active (browser connected). |

## Building the Firmware

1.  **Install SDK:** Ensure you have the Raspberry Pi Pico SDK installed and configured.
2.  **Build:**
    ```bash
    mkdir build
    cd build
    cmake ..
    make
    ```
3.  **Flash:** Hold the `BOOT` button on the RP2040, plug it in, and drag the generated `.uf2` file into the `RPI-RP2` drive.

## Troubleshooting
- Currently when refreshing the web page on WebUSB devices the pico/usb device needs to be reset. Unplugging or pressing reset on the USB adapter should acomplish this
- If on linux you may need to edit Udev rules. See here https://stackoverflow.com/questions/30983221/chrome-app-fails-to-open-usb-device

Print functionality derived from firmware at https://github.com/stacksmashing/gb-link-printer
