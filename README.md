## Required Libraries (Arduino Library Manager)
Open Sketch -> Include Library -> Manage Libraries... (or Ctrl+Shift+I / Cmd+Shift+I) and install the following:
**Required Libraries**

| Library Name in Library Manager | Author | Used For | Notes |
| :--- | :--- | :--- | :--- |
| `Adafruit AS7341` | Adafruit | AS7341 spectral sensor | When prompted, choose **Install All** to automatically install dependencies (`Adafruit BusIO` and `Adafruit Unified Sensor`). |
| `ESP32Servo` | Kevin Harrington / John K. Bennett | Servo motor control on ESP32 | Matches `madhephaestus/ESP32Servo`. |
| `TM1637_RT` | Rob Tillaart | 4-digit 7-segment display | Matches `robtillaart/TM1637_RT` (provides `#include <TM1637.h>`). |

Built-in ESP32 Core Libraries (No installation required)
- LittleFS (#include <LittleFS.h>)
- FS
- Wire

## Board Support Package Setup
1.
In Arduino IDE, go to Settings / Preferences and ensure the ESP32 board URL is in the Additional Boards Manager URLs field:
https://espressif.github.io/arduino-esp32/package_esp32_index.json
2.
Go to Tools -> Board -> Boards Manager..., search for esp32 by Espressif Systems, and install it.
3.
Select your board under Tools -> Board -> esp32 -> ESP32 Dev Module.
4.
Set the Serial Monitor baud rate to 115200 (Tools -> Serial Monitor).

## Spectral Classification

The classifier uses the eight visible AS7341 channels normalized by their total, plus a lightly weighted logarithmic brightness feature. Clear and NIR remain in the serial training log but are not used as color features. Readings with a visible-channel sum below 100 counts or any selected raw channel at or above 18,000 counts are ignored; confirm this provisional saturation cutoff with the sensor and exposure settings in use.

The six-neighbor classifier starts with a maximum feature distance of 0.02 and requires a weighted-vote margin of at least 3. These thresholds are starting points, not calibrated guarantees; validate them with held-out beads and orientations. A rejected or uncertain reading is returned as class 15. Verify that the machine's actuator arrangement routes this value to a safe reject position before sorting.

Training stores up to 30 valid readings per color. The feature layout and capacity changed, so existing `/training.bin` files are rejected; enter training mode and collect new data before normal sorting. The current text dataset does not identify individual beads, so its sample-level validation cannot establish accuracy on unseen beads.
