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
