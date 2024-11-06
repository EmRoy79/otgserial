#include "OTGSerial.h"

static const char* TAG = "OTGSerial";

OTGSerialClass OTGSerial;

uint32_t OTGSerialClass::_baudrate = 9600;
bool OTGSerialClass::_connected = false;
bool OTGSerialClass::_error = false;
Ringbuffer OTGSerialClass::_inBuf(OTG_IN_BUFFER_SIZE);
Ringbuffer OTGSerialClass::_outBuf(OTG_OUT_BUFFER_SIZE);
bool OTGSerialClass::_sending = false;
class_driver_t *OTGSerialClass::_current_driver_obj = NULL; 
usb_transfer_t *OTGSerialClass::_transfer_in = NULL;
uint8_t* OTGSerialClass::_transfer_in_buffer = NULL;


OTGSerialClass::OTGSerialClass() {
}

void OTGSerialClass::begin(unsigned long baudrate) {
    _baudrate = baudrate;
    SemaphoreHandle_t signaling_sem = xSemaphoreCreateBinary();
    assert(signaling_sem != NULL);
    BaseType_t task_ret;
    task_ret = xTaskCreatePinnedToCore(host_lib_daemon_task, "host_lib_daemon_task", 4096, (void *)signaling_sem, OTG_DAEMON_TASK_PRIORITY, NULL, OTG_DAEMON_TASK_CORE);
    assert(task_ret == pdTRUE);
    task_ret = xTaskCreatePinnedToCore(class_driver_task, "class_driver_task", 4096, (void *)signaling_sem, OTG_CLASS_TASK_PRIORITY, NULL, OTG_CLASS_TASK_CORE);
    assert(task_ret == pdTRUE);
    if (_transfer_in == NULL) {
      _transfer_in = (usb_transfer_t*)malloc(sizeof(usb_transfer_t));
    }
}

bool OTGSerialClass::connected() {
    return _connected;
}

bool OTGSerialClass::unsupported() {
    return _error;
}

void OTGSerialClass::println(char* data) {
    OTGSerialClass::printf("%s\n",data);
}

void OTGSerialClass::print(char data) {
  _outBuf.push(data);
  if (!_sending) {
    transmit_buffer();
  }
}

void OTGSerialClass::print(int data) {
  char buffer[12];
  itoa(data, buffer, 10);
  _outBuf.push(buffer, strlen(buffer));
  if (!_sending) {
    transmit_buffer();
  }
}

void OTGSerialClass::print(const char* data) {
  _outBuf.push(data, strlen(data));
  if (!_sending) {
    transmit_buffer();
  }
}

void OTGSerialClass::printf(const char* format, ...) {
  char buffer[OTG_OUT_BUFFER_SIZE];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  _outBuf.push(buffer, strlen(buffer));
  if (!_sending) {
    transmit_buffer();
  }
}

void OTGSerialClass::clearBuffer() {
  _inBuf.clear();
  _outBuf.clear();
}

uint16_t OTGSerialClass::available() {
  return _inBuf.available();
}

uint16_t OTGSerialClass::available(const char delimiter) {
  return _inBuf.available(delimiter);
}

int OTGSerialClass::read() {
  return _inBuf.available()?_inBuf.pop():-1;
}

char* OTGSerialClass::readUntil(const char delimiter) {
  int availableCount = available(delimiter);
  if (availableCount <= 0) {
    char* emptyResult = new char[1];
    emptyResult[0] = '\0';
    return emptyResult;
  }
  char* buffer = new char[availableCount + 1];
  for (uint16_t i = 0; i < availableCount; i++) {
    buffer[i] = (char) _inBuf.pop();
  }
  buffer[availableCount] = '\0';
  return buffer;
}

void OTGSerialClass::host_lib_daemon_task(void *arg) {
  SemaphoreHandle_t signaling_sem = (SemaphoreHandle_t)arg;

  usb_host_config_t host_config = {
      .skip_phy_setup = false,
      .intr_flags = ESP_INTR_FLAG_LEVEL1,
  };
  ESP_ERROR_CHECK(usb_host_install(&host_config));
  xSemaphoreGive(signaling_sem);
  vTaskDelay(10);

  bool has_clients = true;
  bool has_devices = true;
  while (has_clients || has_devices) {
      uint32_t event_flags;
      ESP_ERROR_CHECK(usb_host_lib_handle_events(portMAX_DELAY, &event_flags));
      if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
          has_clients = false;
      }
      if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
          has_devices = false;
      }
  }
  ESP_ERROR_CHECK(usb_host_uninstall());
  vSemaphoreDelete(signaling_sem);
  vTaskDelete(NULL);
}

void OTGSerialClass::class_driver_task(void *arg) {
    SemaphoreHandle_t signaling_sem = (SemaphoreHandle_t)arg;
    class_driver_t driver_obj = {0};
    driver_obj.ep_bulk_in = 0xff;
    driver_obj.ep_size_in = 0xff;
    driver_obj.ep_bulk_out = 0xff;
    driver_obj.ep_size_out = 0xff;
    xSemaphoreTake(signaling_sem, portMAX_DELAY);

    usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = OTG_CLIENT_NUM_EVENT_MSG,
        .async = {
            .client_event_callback = client_event_cb,
            .callback_arg = (void *)&driver_obj,
        },
    };
    ESP_ERROR_CHECK(usb_host_client_register(&client_config, &driver_obj.client_hdl));

    while (1) {
        if (driver_obj.actions == 0) {
            usb_host_client_handle_events(driver_obj.client_hdl, portMAX_DELAY);
        } else {
            if (driver_obj.actions & ACTION_OPEN_DEV) {
                action_open_dev(&driver_obj);
            }
            if (driver_obj.actions & ACTION_CLOSE_DEV) {
                action_close_dev(&driver_obj);
                driver_obj.actions = 0;
            }
            if (driver_obj.actions & ACTION_EXIT) {
                break;
            }
        }
    }

    ESP_ERROR_CHECK(usb_host_client_deregister(driver_obj.client_hdl));

    xSemaphoreGive(signaling_sem);
    vTaskSuspend(NULL);
}

void OTGSerialClass::client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg)
{
    class_driver_t *driver_obj = (class_driver_t *)arg;
    switch (event_msg->event) {
        case USB_HOST_CLIENT_EVENT_NEW_DEV:
            if (driver_obj->dev_addr == 0) {
                driver_obj->dev_addr = event_msg->new_dev.address;
                driver_obj->actions |= ACTION_OPEN_DEV;
                ESP_LOGD(TAG,"New device connected at address: %d\n", driver_obj->dev_addr);
            }
            break;
        case USB_HOST_CLIENT_EVENT_DEV_GONE:
            if (driver_obj->dev_hdl != NULL) {
                driver_obj->actions = ACTION_CLOSE_DEV;
                ESP_LOGD(TAG,"Device disconnected, scheduling close action.");
            }
            break;
        default:
            abort();
    }
}

void OTGSerialClass::receive_buffer_cb(usb_transfer_t *transfer) {
  if (transfer->status == USB_TRANSFER_STATUS_COMPLETED) {
    if (transfer->actual_num_bytes) {
      _inBuf.push((const char*)transfer->data_buffer, transfer->actual_num_bytes);
    }
  }
  vTaskDelay(10 / portTICK_PERIOD_MS);
  usb_host_transfer_submit(_transfer_in);
}

void OTGSerialClass::transmit_buffer() {
    if (!_connected || _current_driver_obj == NULL) {
    ESP_LOGE(TAG, "Device not connected!");
    return;
  }
   _sending = true;

  uint16_t len = min(
      _current_driver_obj->ep_size_out,
      _outBuf.available()
  );

  if (len==0) {
    _sending = false;
    return;
  }

  usb_transfer_t* transfer_out;
  esp_err_t err = usb_host_transfer_alloc(len, 0, &transfer_out);
  if (err != ESP_OK) {
      ESP_LOGE(TAG, "usb_host_transfer_alloc failed: %s", esp_err_to_name(err));
      _sending = false;
      return;
  }

  _outBuf.pop((char*)transfer_out->data_buffer, len);
  transfer_out->device_handle = _current_driver_obj->dev_hdl;
  transfer_out->bEndpointAddress = _current_driver_obj->ep_bulk_out;
  transfer_out->callback = transmit_buffer_cb;
  transfer_out->context = NULL;
  transfer_out->num_bytes = len;
  
  err = usb_host_transfer_submit(transfer_out);
  if (err != ESP_OK) {
      ESP_LOGE(TAG, "usb_host_transfer_submit failed: %s", esp_err_to_name(err));
      usb_host_transfer_free(transfer_out);
      _sending = false;
      return;
  }
}

void OTGSerialClass::transmit_buffer_cb(usb_transfer_t *transfer) {
    if (transfer->status == USB_TRANSFER_STATUS_COMPLETED) {
        usb_host_transfer_free(transfer);
        transmit_buffer();
    } else {
        ESP_LOGE(TAG, "Transfer failed with status %d", transfer->status);
        _outBuf.clear();
        _sending = false;
        usb_host_transfer_free(transfer);
    }
}

esp_err_t OTGSerialClass::vendor_control_in(class_driver_t *driver_obj, uint8_t request, uint16_t value, uint16_t index, uint16_t wLength, usb_transfer_cb_t callback) {
  usb_setup_packet_t setup_packet = {
    .bmRequestType = VENDOR_READ_TYPE,
    .bRequest = request,
    .wValue = value,
    .wIndex = index,
    .wLength = wLength
  };
  
  size_t transfer_size = sizeof(usb_setup_packet_t) + wLength;  
  usb_transfer_t *control_transfer;

  esp_err_t err = usb_host_transfer_alloc(transfer_size, 0, &control_transfer);
  if (err != ESP_OK) {
    return err;
  }
  control_transfer->device_handle = driver_obj->dev_hdl;
  control_transfer->bEndpointAddress = 0;
  control_transfer->num_bytes = transfer_size;
  control_transfer->callback = callback;
  control_transfer->context = driver_obj;

  uint8_t *data_buffer = control_transfer->data_buffer;
  memcpy(data_buffer, &setup_packet, sizeof(usb_setup_packet_t));

  err = usb_host_transfer_submit_control(driver_obj->client_hdl, control_transfer);
  if (err != ESP_OK) {
    usb_host_transfer_free(control_transfer);
  }
  return err;
}

esp_err_t OTGSerialClass::vendor_control_out(class_driver_t *driver_obj, uint8_t request, uint16_t value, uint16_t index, uint8_t* data, uint16_t wLength, usb_transfer_cb_t callback) {
  usb_setup_packet_t setup_packet = {
    .bmRequestType = VENDOR_WRITE_TYPE,
    .bRequest = request,
    .wValue = value,
    .wIndex = index,
    .wLength = wLength
  };
    
  size_t transfer_size = sizeof(usb_setup_packet_t) + wLength;
    
  usb_transfer_t *control_transfer;
  esp_err_t err = usb_host_transfer_alloc(transfer_size, 0, &control_transfer);
  if (err != ESP_OK) {
    return err;
  }

  control_transfer->device_handle = driver_obj->dev_hdl;
  control_transfer->bEndpointAddress = 0;
  control_transfer->num_bytes = transfer_size;
  control_transfer->callback = callback;
  control_transfer->context = driver_obj;

  uint8_t *data_buffer = control_transfer->data_buffer;
  memcpy(data_buffer, &setup_packet, sizeof(usb_setup_packet_t));
  if (wLength > 0 && data != NULL) {
      memcpy(data_buffer + sizeof(usb_setup_packet_t), data, wLength);
  }

  err = usb_host_transfer_submit_control(driver_obj->client_hdl, control_transfer);
  if (err != ESP_OK) {
      usb_host_transfer_free(control_transfer);
  }
  return err;
}

void OTGSerialClass::handle_error(const char* tag, esp_err_t e) {
  ESP_LOGW(TAG,"(E) Error occured while %s: %d\n", tag, e);
  _error=true;
  _connected=false;
}

bool OTGSerialClass::check_serial(uint16_t vendor, uint16_t product) {
  return (vendor==0x1A86 && product==0x7522) 
    || (vendor==0x1A86 && product==0x7523)
  ;
}

int OTGSerialClass::calc_baudrate( uint32_t baud_rate, uint16_t *factor, uint16_t *divisor)
{
	unsigned char a;
	unsigned char b;
	unsigned long c;

	switch ( baud_rate ) {
	case 921600: 
		a = 0xf3; 
		b = 7; 
		break; 
	case 307200:
		a = 0xd9; 
		b = 7; 
		break; 
	default: 
		if ( baud_rate > 6000000/255 ) { 
			b = 3;
			c = 6000000;
		} else if ( baud_rate > 750000/255 ) {  
			b = 2;
			c = 750000;
		} else if (baud_rate > 93750/255) { 
			b = 1;
			c = 93750;
		} else {
			b = 0;
			c = 11719;
		}
		a = (unsigned char)(c / baud_rate);
		if (a == 0 || a == 0xFF) return -EINVAL;
		if ((c / a - baud_rate) > (baud_rate - c / (a + 1))) 
			a ++;
		a = 256 - a;
		break;
	}
	*factor = a;
	*divisor = b;
	return 0;
}

void OTGSerialClass::action_open_dev(class_driver_t *driver_obj) {
  _connected = false;
  _error = false;
  assert(driver_obj->dev_addr != 0);
  ESP_ERROR_CHECK_HANDLE(
    "usb_host_device_open",
    usb_host_device_open(driver_obj->client_hdl, driver_obj->dev_addr, &driver_obj->dev_hdl), 
    handle_error
  );
  assert(driver_obj->dev_hdl != NULL);

  const usb_device_desc_t *dev_desc;
  ESP_ERROR_CHECK_HANDLE(
    "usb_host_get_device_descriptor",
    usb_host_get_device_descriptor(driver_obj->dev_hdl, &dev_desc),
    handle_error
  );
  if (!check_serial(dev_desc->idVendor,dev_desc->idProduct)) {
    ESP_LOGW(TAG,"Unsupported device");
    return;
  }

  const usb_config_desc_t *config_desc;
  ESP_ERROR_CHECK_HANDLE(
    "usb_host_get_active_config_descriptor",
    usb_host_get_active_config_descriptor(driver_obj->dev_hdl, &config_desc), 
    handle_error
  );

  assert(config_desc != NULL);
  int offset = 0;
  uint16_t wTotalLength = config_desc->wTotalLength;

  const usb_standard_desc_t *next_desc = (const usb_standard_desc_t *)config_desc;
  do {
    if (next_desc->bDescriptorType==USB_B_DESCRIPTOR_TYPE_INTERFACE) {
      const usb_intf_desc_t *intf = (const usb_intf_desc_t *)next_desc;
      driver_obj->ep_bulk_in = 0xff;
      driver_obj->ep_size_in = 0xff;
      driver_obj->ep_bulk_out = 0xff;
      driver_obj->ep_size_out = 0xff;
      driver_obj->intf = *intf;
    } else
    if (next_desc->bDescriptorType==USB_B_DESCRIPTOR_TYPE_ENDPOINT) {
      const usb_ep_desc_t *ep_desc = (const usb_ep_desc_t *)next_desc;
      if ((ep_desc->bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK) == USB_BM_ATTRIBUTES_XFER_BULK) {
        if (USB_EP_DESC_GET_EP_DIR(ep_desc)) {
          driver_obj->ep_bulk_in = ep_desc->bEndpointAddress;
          driver_obj->ep_size_in = ep_desc->wMaxPacketSize;
        } else {
          driver_obj->ep_bulk_out = ep_desc->bEndpointAddress;
          driver_obj->ep_size_out = ep_desc->wMaxPacketSize;
        }
      } 
    }
    next_desc = usb_parse_next_descriptor(next_desc, wTotalLength, &offset);
  } while (next_desc != NULL && (driver_obj->ep_bulk_in==0xff || driver_obj->ep_bulk_out==0xff));

  driver_obj->actions &= ~ACTION_OPEN_DEV;

  if (driver_obj->ep_bulk_in == 0xff || driver_obj->ep_bulk_out == 0xff) {
    ESP_LOGW(TAG,"(E) No IN/OUT detected");
    _error = true;
    return;
  }      

  ESP_ERROR_CHECK_HANDLE(
    "usb_host_interface_claim",
    usb_host_interface_claim(
      driver_obj->client_hdl,
      driver_obj->dev_hdl,
      driver_obj->intf.bInterfaceNumber,
      driver_obj->intf.bAlternateSetting
    ), 
    handle_error
  );

  _transfer_in_buffer = (uint8_t*)malloc(driver_obj->ep_size_in);

  ESP_ERROR_CHECK_HANDLE(
    "allocate in buffer",
    usb_host_transfer_alloc(driver_obj->ep_size_in, 0, &_transfer_in),
    handle_error
  );

  if (_transfer_in_buffer == NULL) {
      ESP_LOGE(TAG, "Failed to allocate transfer buffer");
      return;
  }
  _transfer_in->device_handle = driver_obj->dev_hdl;
  _transfer_in->bEndpointAddress = driver_obj->ep_bulk_in;
  _transfer_in->callback = OTGSerialClass::receive_buffer_cb;
  _transfer_in->context = (class_driver_t *)driver_obj;
  _transfer_in->num_bytes = driver_obj->ep_size_in;

  ESP_ERROR_CHECK_HANDLE(
    "usb_host_transfer_submit",
    usb_host_transfer_submit(_transfer_in), 
    handle_error
  );
  chain_get_version(driver_obj);
}

void OTGSerialClass::chain_get_version(class_driver_t *driver_obj) {
  ESP_ERROR_CHECK_HANDLE(
    "chain_get_version",
    vendor_control_in(driver_obj, VENDOR_VERSION, 0, 0, 2, chain_get_version_cb),
    handle_error
  );
}

void OTGSerialClass::chain_get_version_cb(usb_transfer_t *transfer) {
  if (!transfer->status == USB_TRANSFER_STATUS_COMPLETED || transfer->num_bytes != 10) {
    usb_host_transfer_free(transfer);
    ESP_LOGW(TAG,"(E) chain failed at version: %d\n", transfer->status);
    _error=true;
    return;
  }
  ESP_LOGD(TAG,"Version: %d.%d\n", transfer->data_buffer[6], transfer->data_buffer[7]);
  usb_host_transfer_free(transfer);
  chain_init_serial((class_driver_t *)transfer->context);
}

void OTGSerialClass::chain_init_serial(class_driver_t *driver_obj) {
  ESP_ERROR_CHECK_HANDLE(
    "chain_init_serial",
    vendor_control_out(driver_obj, VENDOR_SERIAL_INIT, 0, 0, NULL, 0, chain_init_serial_cb),
    handle_error
  );
}

void OTGSerialClass::chain_init_serial_cb(usb_transfer_t *transfer) {
  if (!transfer->status == USB_TRANSFER_STATUS_COMPLETED || transfer->num_bytes != 8) {
    usb_host_transfer_free(transfer);
    ESP_LOGW(TAG,"(E) chain failed at chain_init_serial_cb: %d\n", transfer->status);
    _error=true;
    return;
  }
  usb_host_transfer_free(transfer);
  chain_set_write_1((class_driver_t *)transfer->context);
}

void OTGSerialClass::chain_set_write_1(class_driver_t *driver_obj) {
  ESP_ERROR_CHECK_HANDLE(
    "chain_set_write_1",
    vendor_control_out(driver_obj, VENDOR_WRITE, 0x0F2C, 0x0004, NULL, 0, chain_set_write_1_cb),
    handle_error
  );
}

void OTGSerialClass::chain_set_write_1_cb(usb_transfer_t *transfer) {
  if (!transfer->status == USB_TRANSFER_STATUS_COMPLETED || transfer->num_bytes != 8) {
    usb_host_transfer_free(transfer);
    ESP_LOGW(TAG,"(E) chain failed at chain_set_write_1_cb: %d\n", transfer->status);
    _error=true;
    return;
  }
  usb_host_transfer_free(transfer);
  chain_set_write_2((class_driver_t *)transfer->context);
}

void OTGSerialClass::chain_set_write_2(class_driver_t *driver_obj) {
  ESP_ERROR_CHECK_HANDLE(
    "chain_set_write_2",
    vendor_control_out(driver_obj, VENDOR_WRITE, 0x2518, 0x0000, NULL, 0, chain_set_write_2_cb),
    handle_error
  );
}

void OTGSerialClass::chain_set_write_2_cb(usb_transfer_t *transfer) {
  if (!transfer->status == USB_TRANSFER_STATUS_COMPLETED || transfer->num_bytes != 8) {
    usb_host_transfer_free(transfer);
    ESP_LOGW(TAG,"(E) chain failed at chain_write_2_cb: %d\n", transfer->status);
    _error=true;
    return;
  }
  usb_host_transfer_free(transfer);
  chain_get_read((class_driver_t *)transfer->context);
}

void OTGSerialClass::chain_get_read(class_driver_t *driver_obj) {
  ESP_ERROR_CHECK_HANDLE(
    "chain_get_read",
    vendor_control_in(driver_obj, VENDOR_WRITE, 0x2518, 0x0000, 2,  chain_get_read_cb),
    handle_error
  );
}

void OTGSerialClass::chain_get_read_cb(usb_transfer_t *transfer) {
  if (!transfer->status == USB_TRANSFER_STATUS_COMPLETED || transfer->num_bytes != 10) {
    usb_host_transfer_free(transfer);
    ESP_LOGW(TAG,"(E) chain failed at chain_get_read_cb: %d\n", transfer->status);
    _error=true;
    return;
  }
  usb_host_transfer_free(transfer);
  chain_set_baudrate((class_driver_t *)transfer->context);

}

void OTGSerialClass::chain_set_baudrate(class_driver_t *driver_obj) {
	uint16_t divisor = 0;
	uint16_t factor = 0;
	calc_baudrate(_baudrate, &factor, &divisor );	
	uint16_t value =  0x9c | ((OTG_SERIAL_SETTINGS | 0xc0 )<< 8);
	uint16_t index = 0x08 | divisor | (factor << 8);

  ESP_ERROR_CHECK_HANDLE(
    "chain_init_serial",
    vendor_control_out(driver_obj, VENDOR_SERIAL_INIT, value, index, NULL, 0, chain_set_baudrate_cb),
    handle_error
  );
}

void OTGSerialClass::chain_set_baudrate_cb(usb_transfer_t *transfer) {
  if (!transfer->status == USB_TRANSFER_STATUS_COMPLETED || transfer->num_bytes != 8) {
    usb_host_transfer_free(transfer);
    ESP_LOGW(TAG,"(E) chain failed at chain_set_baudrate_cb: %d\n", transfer->status);
    _error=true;
    return;
  }
  usb_host_transfer_free(transfer);
  chain_set_hw_control((class_driver_t *)transfer->context);
}

void OTGSerialClass::chain_set_hw_control(class_driver_t *driver_obj) {
#ifndef OTG_HW_CONTROL
    ESP_LOGD(TAG,"HW_Control deactivated");
    chain_set_modem_out(driver_obj);
    return;
#else
  ESP_ERROR_CHECK_HANDLE(
    "chain_set_hw_control",
    vendor_control_out(driver_obj, VENDOR_WRITE, 0x2727, 0x0101, NULL, 0, chain_set_hw_control_cb),
    handle_error
  );
#endif
}

void OTGSerialClass::chain_set_hw_control_cb(usb_transfer_t *transfer) {
  if (!transfer->status == USB_TRANSFER_STATUS_COMPLETED || transfer->num_bytes != 8) {
    usb_host_transfer_free(transfer);
    ESP_LOGW(TAG,"(E) chain failed at chain_set_hw_control_cb: %d\n", transfer->status);
    _error=true;
    return;
  }
  usb_host_transfer_free(transfer);
  chain_set_modem_out((class_driver_t *)transfer->context);
}

void OTGSerialClass::chain_set_modem_out(class_driver_t *driver_obj) {
  ESP_ERROR_CHECK_HANDLE(
    "chain_set_hw_control",
    vendor_control_out(driver_obj, VENDOR_MODEM_OUT, 0x2727, 0x0000, NULL, 0, chain_set_modem_out_cb),
    handle_error
  );
}

void OTGSerialClass::chain_set_modem_out_cb(usb_transfer_t *transfer) {
  if (!transfer->status == USB_TRANSFER_STATUS_COMPLETED || transfer->num_bytes != 8) {
    usb_host_transfer_free(transfer);
    ESP_LOGW(TAG,"(E) chain failed at chain_set_modem_out_cb out: %d\n", transfer->status);
    _error=true;
    return;
  }
  _current_driver_obj = (class_driver_t *)transfer->context;
  ESP_LOGD(TAG,"Successfully initialized!");
  _connected=true;
  usb_host_transfer_free(transfer);
}

void OTGSerialClass::action_close_dev(class_driver_t *driver_obj) {
  if (driver_obj->client_hdl != NULL && driver_obj->dev_hdl != NULL) {
    esp_err_t release_err = usb_host_interface_release(driver_obj->client_hdl, driver_obj->dev_hdl, driver_obj->intf.bInterfaceNumber);
    if (release_err != ESP_OK) {
      ESP_LOGW(TAG,"Error releasing interface: %s",esp_err_to_name(release_err));
    }    
    esp_err_t err = usb_host_device_close(driver_obj->client_hdl, driver_obj->dev_hdl);
    if (err != ESP_OK) {
      ESP_LOGW(TAG,"Error closing device: %s",esp_err_to_name(err));
    }
    if (_transfer_in != NULL) {
      usb_host_transfer_free(_transfer_in);
    }

    driver_obj->ep_bulk_in = 0xff;
    driver_obj->ep_size_in = 0xff;
    driver_obj->ep_bulk_out = 0xff;
    driver_obj->ep_size_out = 0xff;
    driver_obj->dev_hdl = NULL;
    driver_obj->dev_addr = 0;
    _current_driver_obj = NULL;
  } else {
    ESP_LOGW(TAG,"Invalid handle(s), device already closed or not initialized.");
  }
    
  driver_obj->actions = 0;
}






