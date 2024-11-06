
# OTGSerial Library

## Overview

The OTGSerial library provides serial communication functionality for ESP32 devices with OTG (On-The-Go) ports. This library allows for seamless data transmission and reception over USB, making it ideal for applications requiring robust serial communication between an ESP32 and other USB devices.

## Features

- **Configurable Buffer Sizes:** Set custom buffer sizes for inbound and outbound data.
- **Flexible Serial Settings:** Customize data bits, parity, and stop bits for OTG communication.
- **Task Management:** Separate core and priority settings for OTG daemon and class tasks.
- **Event Handling:** Efficient handling of USB host events and device actions.
- **Ring Buffer Implementation:** Robust ring buffer for efficient data storage and retrieval.
- **Error Handling:** Comprehensive error checking and handling mechanisms.

## Installation

1. Download the library files and place them in your Arduino libraries folder.
2. Include the library in your Arduino sketch:
   ```cpp
   #include <OTGSerial.h>
   ```

## Precompiler Definitions

You can customize various settings of the OTGSerial library using precompiler definitions. These definitions should be placed at the top of your sketch before including the OTGSerial library:

| Name                     | Description                                         | Default Value |
|--------------------------|-----------------------------------------------------|---------------|
| `OTG_IN_BUFFER_SIZE`     | Define the size of the OTG inbound buffer.          | 1024 bytes    |
| `OTG_OUT_BUFFER_SIZE`    | Define the size of the OTG outbound buffer.         | 256 bytes     |
| `OTG_SERIAL_SETTINGS`    | Define the serial settings for OTG communication.   | CS8 \| NONE \| STOP1 |
| `OTG_DAEMON_TASK_CORE`   | Define the core for the OTG daemon task.            | 0             |
| `OTG_DAEMON_TASK_PRIORITY`| Define the priority for the OTG daemon task.       | 2             |
| `OTG_CLASS_TASK_CORE`    | Define the core for the OTG class task.             | 1             |
| `OTG_CLASS_TASK_PRIORITY`| Define the priority for the OTG class task.         | 3             |
| `OTG_CLIENT_NUM_EVENT_MSG`| Define the number of event messages for the OTG client. | 5             |
| `OTG_HW_CONTROL`         | Optional hardware control definition. Uncomment to enable hardware control features. | Not defined   |



## Usage

### Basic Example

This example demonstrates how to use the OTGSerial library for serial communication:

```cpp
#include <OTGSerial.h>

#define TIMEOUT 100

char inputBuffer[256];
unsigned long lastReceiveTime = 0;
int bufferIndex = 0;

void setup() {
  Serial.begin(115200);
  OTGSerial.begin(9600);
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (bufferIndex < sizeof(inputBuffer) - 1) {
      inputBuffer[bufferIndex++] = c;
    }
    lastReceiveTime = millis();
  }

  if (millis() - lastReceiveTime > TIMEOUT && bufferIndex > 0) {
    inputBuffer[bufferIndex] = '\0';
    OTGSerial.print(inputBuffer);
    bufferIndex = 0;
  }

  uint16_t available = OTGSerial.available('\n');
  if (available > 0) {
    char* line = OTGSerial.readUntil('\n');
    Serial.printf("(R) %d %s", available, line);
    free(line);
  }
  Serial.println(esp_get_free_heap_size());

  delay(10);
}
```

### API

- `void begin(unsigned long baudrate)`: Initialize the OTGSerial communication with the specified baud rate.
- `bool connected()`: Check if the OTGSerial is connected.
- `bool unsupported()`: Check if the connected device is unsupported.
- `void print(char data)`: Send a single character.
- `void print(const char* data)`: Send a string.
- `void print(int data)`: Send an integer.
- `void printf(const char* format, ...)`: Send formatted data.
- `void clearBuffer()`: Clear the buffer.
- `uint16_t available()`: Get the number of bytes available in the buffer.
- `uint16_t available(const char delimiter)`: Get the number of bytes available until the delimiter.
- `int read()`: Read a single byte from the buffer.
- `char* readUntil(const char delimiter)`: Read data from the buffer until the delimiter.

## Known Issues

- On every disconnect/connect of the OTG device, there is a small memory leak of a few bytes. This leak is considered acceptable for my purposes.

## License

MIT License

Written by EmRoy79

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
