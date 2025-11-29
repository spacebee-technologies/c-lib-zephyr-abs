#include "zephyr_usb_cdc.h"

#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>

//******************************************************************************
// Interface implementations
//******************************************************************************
static uint8_t _send(void *self, const uint8_t *buffer, size_t bufferSize) {
  ZephyrUsbCdc *_self = (ZephyrUsbCdc *)self;

  for (size_t i = 0; i < bufferSize; i++) {
    uart_poll_out(_self->usbDev, buffer[i]);
  }

  return 0;
}

static uint8_t _receive(void *self, uint8_t *buffer, size_t bufferSize, size_t *receivedSize) {
  ZephyrUsbCdc *_self = (ZephyrUsbCdc *)self;

  *receivedSize = 0;
  uint8_t byte;
  for (size_t i = 0; i < bufferSize; i++) {
    int ret = uart_poll_in(_self->usbDev, &byte);
    if (ret == 0) {
      buffer[i] = byte;
      (*receivedSize)++;
    } else if (ret == -1) {
      break;  // No more data available
    } else {
      return ret;
    }
  }

  return 0;
}

//******************************************************************************
// Public methods
//******************************************************************************
void ZephyrUsbCdc_create(ZephyrUsbCdc *self, const struct device *usbDev) {
  // Initialize interface view
  self->communicationInterfaceView.instance = self;
  self->communicationInterfaceView.send = _send;
  self->communicationInterfaceView.receive = _receive;

  self->usbDev = usbDev;

  // Initialize USB
  int ret = usb_enable(NULL);
  if (ret != 0) {
    return;
  }

  // Wait for USB enumeration
  k_sleep(K_MSEC(1000));

  // Check if USB device is ready
  if (!device_is_ready(self->usbDev)) {
    return;
  }
}

CommunicationInterface *ZephyrUsbCdc_viewAsCommunicationInterface(ZephyrUsbCdc *self) {
  return &self->communicationInterfaceView;
}
