# ESP32-S3 Print Server

Turns a USB printer into a WiFi printer. Plug the printer into an ESP32-S3,
power it up, and it joins your WiFi and serves a dashboard in the browser.

Based on [WARMW00D/ESP32-S3-PrintServer](https://github.com/WARMW00D/ESP32-S3-PrintServer)
(MIT license, see `LICENSE`), ported to PlatformIO and reworked.

## What it does

- **Print from a computer**: Raw/JetDirect on port 9100 and LPR on port 515.
  The computer's own printer driver makes the data; the board just passes it
  to the printer, so this works with almost any USB printer.
- **Print from Android** (IPP on port 631): the phone sends page images and
  the board converts them for the printer.
  - Epson inkjets (ESC/P-R): colour or grey, A4, 300 dpi, one-sided.
  - HP-style lasers (PCL 5): black and white, A4, 300/600 dpi, duplex.
  - Other printers: phone printing is refused; computer printing still works.
- **Dashboard** at the board's IP: printer status, USB details, WiFi, memory,
  setup steps for Windows/macOS/Android, WiFi reset and an optional password.
- The printer's USB Device ID decides its language, so status polling (PJL)
  is only used on printers that support it.

## Hardware

- ESP32-S3 board with 16 MB flash and 8 MB OPI PSRAM (e.g. "N16R8"),
  ideally with two USB-C ports.
- **OTG/USB port** → printer. **UART/COM port** → your PC for flashing and logs.
- Built-in RGB LED on GPIO48 and the BOOT button on GPIO0 (change
  `RGB_LED_PIN` / `RESET_BUTTON_PIN` in `src/main.cpp` if your board differs).

## Setup

1. Put your WiFi name and password in `src/main.cpp`:
   ```cpp
   const char *WIFI_SSID     = "YOUR_WIFI_NAME";
   const char *WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
   ```
   Don't publish the file with these filled in.
2. Flash through the UART/COM port: `pio run -t upload`
3. Plug in the printer and power the board. Find its IP in your router's
   device list, at `http://ESP32-PrintServer.local`, or with
   `pio device monitor`. Open it in a browser.
4. **Computer**: install the printer's normal driver and add a TCP/IP printer
   at that IP, Raw port 9100.
   **Android**: enable Default Print Service or Mopria and pick
   "ESP32 Print Server" (or add the IP manually).

If the board can't reach your WiFi it starts its own network
`ESP32-PrintServer` (password `12345678`); connect and open `http://192.168.4.1`
to set WiFi there. Holding BOOT for 10 seconds erases the saved WiFi.

## LED

| Colour | Meaning |
|---|---|
| Green/red alternating | connecting to WiFi |
| Blue, blinking | setup access point mode |
| Red, solid | printer off or not plugged in |
| Green, solid | ready |
| Green, fast blink | printing |
| Yellow, blinking | printer reports an error |
| Red, blinking | reset button held |

## Limits

- Phone printing on an untested printer model may not work; check the serial
  log line `[USB] PJL status: ..., phone printing: ...` after plugging it in.
- The portal password does not protect port 631; anyone on the network can
  send a phone job.
- "Completed" on the phone means the data reached the printer over USB, not
  that paper came out.

## Development

- Board, core and library versions are pinned in `platformio.ini`
  (pioarduino, Arduino core 3.3.9; core 3.3.11 breaks USB host on the S3).
- Host tests for the IPP and raster code: `bash tests/run.sh`
- Query the board or print a test page from a PC:
  `python3 tools/ipp_smoke.py <board-ip> [--print --dpi 300]`
  (Epson printers need `--dpi 300`; the default 600 is for PCL printers.)
