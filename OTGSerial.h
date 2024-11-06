/*
MIT License

Written by EmRoy79

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

// Configuration Block for OTG Serial Settings

// Define the size of the OTG inbound buffer. If not previously defined, set it to 1024 bytes.
#ifndef OTG_BUFFER_SIZE
  #define OTG_IN_BUFFER_SIZE 1024
#endif

// Define the size of the OTG outbound buffer. If not previously defined, set it to 256 bytes.
#ifndef OTG_OUT_BUFFER_SIZE
#define OTG_OUT_BUFFER_SIZE 256
#endif

// Define the serial settings for OTG communication. If not previously defined, 
// use CS8 (8 data bits), NONE (no parity), and STOP1 (1 stop bit).
#ifndef OTG_SERIAL_SETTINGS
  #define OTG_SERIAL_SETTINGS CS8 | NONE | STOP1
#endif

// Define the core for the OTG daemon task. If not previously defined, set it to run on core 0.
#ifndef OTG_DAEMON_TASK_CORE
  #define OTG_DAEMON_TASK_CORE        0
#endif

// Define the priority for the OTG daemon task. If not previously defined, set it to priority level 2.
#ifndef OTG_DAEMON_TASK_PRIORITY
  #define OTG_DAEMON_TASK_PRIORITY    2
#endif

// Define the core for the OTG class task. If not previously defined, set it to run on core 1.
#ifndef OTG_CLASS_TASK_CORE
  #define OTG_CLASS_TASK_CORE         1
#endif

// Define the priority for the OTG class task. If not previously defined, set it to priority level 3.
#ifndef OTG_CLASS_TASK_PRIORITY
  #define OTG_CLASS_TASK_PRIORITY     3
#endif

// Define the number of event messages for the OTG client. If not previously defined, set it to 5.
#ifndef OTG_CLIENT_NUM_EVENT_MSG
  #define OTG_CLIENT_NUM_EVENT_MSG    5
#endif

// Optional hardware control definition. Uncomment to enable hardware control features.
// #define OTG_HW_CONTROL

#ifndef OTGSERIAL_H
#define OTGSERIAL_H

#include <Arduino.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "usb/usb_host.h"
#include "freertos/task.h"
#include "esp_intr_alloc.h"

#define VENDOR_WRITE_TYPE       0x40
#define VENDOR_READ_TYPE        0xC0
#define VENDOR_READ             0x95
#define VENDOR_WRITE            0x9A
#define VENDOR_SERIAL_INIT      0xA1
#define VENDOR_MODEM_OUT        0xA4
#define VENDOR_VERSION          0x5F

#define ACTION_OPEN_DEV         0x01
#define ACTION_CLOSE_DEV        0x04
#define ACTION_EXIT             0x80

#define CS5                     0x00
#define CS6                     0x01
#define CS7                     0x02
#define CS8                     0x03

#define MARK                    0x28
#define SPACE    			          0x38
#define ODD               			0x08
#define EVEN                    0x1B
#define NONE                    0x00

#define STOP1                   0x00
#define STOP2                   0x04

#define ESP_ERROR_CHECK_HANDLE(tag, func, err_func) \
    do { \
        esp_err_t err_code = (func); \
        if (err_code != ESP_OK) { \
            err_func(tag, err_code); \
            return; \
        } \
    } while (0)

struct Ringbuffer {
    Ringbuffer(uint16_t size) : size(size), head(0), tail(0), count(0) {
        buffer = new char[size];
    }

    ~Ringbuffer() {
        delete[] buffer;
    }

    void push(char value) {
        if (count < size) {
            buffer[head] = value;
            head = (head + 1) % size;
            ++count;
        }
    }

    void push(const char* data, uint16_t len) {
      if (count+len < size) {
        for (uint16_t idx = 0; idx < len; idx++) {
          buffer[head] = data[idx];
          head = (head + 1) % size;
          ++count;
        }
      }
    }

    char pop() {
        if (count > 0) {
            uint8_t value = buffer[tail];
            tail = (tail + 1) % size;
            --count;
            return value;
        }
        return 0;
    }

    char* pop(const char delimiter) {
      uint16_t len = available(delimiter);
      char* fragment = new char[len+1];
      for (uint16_t idx=0; idx < len; idx++) {
        fragment[idx]=pop();
      }
      fragment[len+1]='\0';
      return fragment; 
    }

    void pop(char* dest, uint16_t len) {
        for (uint16_t i = 0; i < len && count > 0; ++i) {
            dest[i] = pop();
        }
    }

    uint16_t available() const {
        return count;
    }

    uint16_t available(const char delimiter) {
      for (uint16_t idx = 0; idx < count; idx++) {
        if (buffer[(tail+idx)%size]==delimiter) {
          return idx+1;
        }
      }
      return 0;
    }

    void clear() {
        head = tail = count = 0;
    }

private:
    uint16_t size;
    uint16_t head;
    uint16_t tail;
    uint16_t count;
    char* buffer;
};

typedef struct {
    usb_host_client_handle_t client_hdl;
    uint8_t dev_addr;
    usb_device_handle_t dev_hdl;
    usb_intf_desc_t intf;
    uint8_t ep_bulk_in;
    uint8_t ep_bulk_out;
    uint16_t ep_size_in;
    uint16_t ep_size_out;
    uint32_t actions;
} class_driver_t;

class OTGSerialClass {
public:
    OTGSerialClass();
    static void begin(unsigned long baudrate);
    static bool connected();
    static bool unsupported();
    static void println(char* data);
    static void print(char data);
    static void print(const char* data);
    static void print(int data);
    static void printf(const char* format, ...); 
    static void clearBuffer();
    static uint16_t available();
    static uint16_t available(const char delimiter);
    static int read();
    static char* readUntil(const char delimiter);

private:  
  static uint32_t _baudrate;
  static bool _connected;
  static bool _error;
  static Ringbuffer _inBuf;
  static Ringbuffer _outBuf;
  static bool _sending;
  static class_driver_t *_current_driver_obj;
  static usb_transfer_t *_transfer_in;
  static uint8_t* _transfer_in_buffer;

  static void client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg);
  static void receive_buffer_cb(usb_transfer_t *transfer);
  static void transmit_buffer();
  static void transmit_buffer_cb(usb_transfer_t *transfer);
  static esp_err_t vendor_control_in(class_driver_t *driver_obj, uint8_t request, uint16_t value, uint16_t index, uint16_t wLength, usb_transfer_cb_t callback);
  static esp_err_t vendor_control_out(class_driver_t *driver_obj, uint8_t request, uint16_t value, uint16_t index, uint8_t* data, uint16_t wLength, usb_transfer_cb_t callback);
  
  static void handle_error(const char* tag, esp_err_t e);
  static bool check_serial(uint16_t vendor, uint16_t product);
  static int calc_baudrate( uint32_t baud_rate, uint16_t *factor, uint16_t *divisor);

  static void action_open_dev(class_driver_t *driver_obj);
  static void action_close_dev(class_driver_t *driver_obj);

  static void class_driver_task(void *arg);
  static void host_lib_daemon_task(void *arg);

  static void chain_get_version(class_driver_t *driver_obj);
  static void chain_get_version_cb(usb_transfer_t *transfer);
  static void chain_init_serial(class_driver_t *driver_obj);
  static void chain_init_serial_cb(usb_transfer_t *transfer);
  static void chain_set_write_1(class_driver_t *driver_obj);
  static void chain_set_write_1_cb(usb_transfer_t *transfer);
  static void chain_set_write_2(class_driver_t *driver_obj);
  static void chain_set_write_2_cb(usb_transfer_t *transfer);
  static void chain_get_read(class_driver_t *driver_obj);
  static void chain_get_read_cb(usb_transfer_t *transfer);
  static void chain_set_baudrate(class_driver_t *driver_obj);
  static void chain_set_baudrate_cb(usb_transfer_t *transfer);
  static void chain_set_hw_control(class_driver_t *driver_obj);
  static void chain_set_hw_control_cb(usb_transfer_t *transfer);
  static void chain_set_modem_out(class_driver_t *driver_obj);
  static void chain_set_modem_out_cb(usb_transfer_t *transfer);
};

extern OTGSerialClass OTGSerial;

#endif // OTGSERIAL_H