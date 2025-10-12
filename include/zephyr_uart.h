#ifndef ZEPHYR_UART_H_
#define ZEPHYR_UART_H_

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include "communication_interface.h"
#include "semaphore_interface.h"

#define MSG_SIZE 32
#define MSGQ_ITEMS 10

typedef struct ZephyrUart {
  CommunicationInterface communicationInterfaceView;
  const struct device *dev;
  char rx_buf[MSG_SIZE];
  int rx_buf_pos;
  struct k_msgq msgq;
  uint8_t msgq_buffer[MSGQ_ITEMS * MSG_SIZE];
  SemaphoreInterface *sem;
} ZephyrUart;

uint8_t ZephyrUart_create(ZephyrUart *self, const struct device *dev, SemaphoreInterface *sem);
uint8_t ZephyrUart_configure(ZephyrUart *self);
CommunicationInterface *ZephyrUart_viewAsCommunicationInterface(ZephyrUart *self);

#endif  // ZEPHYR_UART_H_
