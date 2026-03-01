#ifndef ZEPHYR_UART_BIN_H_
#define ZEPHYR_UART_BIN_H_

#include <zephyr/device.h>
#include <zephyr/sys/ring_buffer.h>

#include "communication_interface.h"

#define ZEPHYR_UART_BIN_RING_SIZE 2048

typedef struct ZephyrUartBin {
  CommunicationInterface communicationInterfaceView;
  const struct device *dev;

  struct ring_buf rb;
  uint8_t rbBuf[ZEPHYR_UART_BIN_RING_SIZE];

  volatile uint32_t rxDrops;  // diagnostic
} ZephyrUartBin;

uint8_t ZephyrUartBin_create(ZephyrUartBin *self, const struct device *dev);
CommunicationInterface *ZephyrUartBin_viewAsCommunicationInterface(ZephyrUartBin *self);

#endif  // ZEPHYR_UART_BIN_H_
