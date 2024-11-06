/*
  This example demonstrates the use of the OTGSerial library to handle serial communication
  between a computer and a microcontroller. It is designed to work specifically on ESP32 devices 
  with an OTG (On-The-Go) port.

  Key Features:
  1. Buffering Serial Input:
     - Reads characters from the serial monitor and stores them in an inputBuffer.
     - Characters are accumulated until a specified TIMEOUT period (100 milliseconds) elapses
       without new characters being received.
     - Once the timeout period is reached, the buffer is considered complete, null-terminated,
       and sent via OTGSerial.

  2. Sending Data to OTGSerial:
     - Data accumulated in inputBuffer is sent to the OTGSerial interface.
     - This allows communication with another device connected to OTGSerial.

  3. Receiving Data from OTGSerial:
     - Checks for available data on OTGSerial, specifically looking for lines terminated by '\n'.
     - When a line is available, it reads the line, prints it to the serial monitor, and frees the memory.

  Note:
  - This example is intended to be used on ESP32 devices with an OTG port.

  Known Issues:
  - On every disconnect/connect of the OTG device, there is a small memory leak of a few bytes.
    This leak is considered acceptable for my purposes.
*/

#include <OTGSerial.h>

#define TIMEOUT 100

char inputBuffer[256];
unsigned long lastReceiveTime = 0;
int bufferIndex = 0;

void setup() {
  Serial.begin(115200);
  //OTGSerial.begin(250000);
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
  //Serial.println(esp_get_free_heap_size());

  delay(10);
}
