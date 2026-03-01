#ifndef ZEPHYR_USB_CDC_H_
#define ZEPHYR_USB_CDC_H_

#include "communication_interface.h"

#include <zephyr/device.h>

typedef struct ZephyrUsbCdc {
  CommunicationInterface communicationInterfaceView;
  const struct device *usbDev;
} ZephyrUsbCdc;

/**
 * @brief Constructor for USB CDC structure
 *
 * @param self Uninitialized USB CDC structure
 */
void ZephyrUsbCdc_create(ZephyrUsbCdc *self, const struct device *usbDev);

/**
 * @brief Returns a USB CDC view as a communication interface
 * 
 * @param self Initialized USB CDC structure
 * @return CommunicationInterface* Communication interface view
 */
CommunicationInterface *ZephyrUsbCdc_viewAsCommunicationInterface(ZephyrUsbCdc *self);

#endif  // ZEPHYR_USB_CDC_H_
