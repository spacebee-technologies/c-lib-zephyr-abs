#include "zephyr_uart_bin.h"

#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>

static void uart_rx_isr(const struct device *dev, void *user_data) {
  ZephyrUartBin *self = (ZephyrUartBin *)user_data;

  if (!uart_irq_update(dev)) {
    return;
  }

  if (!uart_irq_rx_ready(dev)) {
    return;
  }

  uint8_t tmp[64];
  int rd;

  while ((rd = uart_fifo_read(dev, tmp, sizeof(tmp))) > 0) {
    uint32_t written = ring_buf_put(&self->rb, tmp, rd);
    if (written < rd) {
      self->rxDrops += (rd - written);
    }
  }
}

static uint8_t _send(void *instance, const uint8_t *buffer, size_t bufferSize) {
  ZephyrUartBin *self = (ZephyrUartBin *)instance;

  for (size_t i = 0; i < bufferSize; i++) {
    uart_poll_out(self->dev, buffer[i]);
  }
  return 0;
}

static uint8_t _receive(void *instance, uint8_t *buffer, size_t bufferSize, size_t *receivedSize) {
  ZephyrUartBin *self = (ZephyrUartBin *)instance;

  if (!receivedSize) {
    return 1;
  }

  uint32_t got = ring_buf_get(&self->rb, buffer, bufferSize);
  *receivedSize = got;

  return 0;
}

uint8_t ZephyrUartBin_create(ZephyrUartBin *self, const struct device *dev) {
  if (!device_is_ready(dev)) {
    return 1;
  }

  self->communicationInterfaceView.instance = self;
  self->communicationInterfaceView.send = _send;
  self->communicationInterfaceView.receive = _receive;

  self->dev = dev;
  self->rxDrops = 0;

  ring_buf_init(&self->rb, sizeof(self->rbBuf), self->rbBuf);

  // Install IRQ RX handler
  uart_irq_callback_user_data_set(dev, uart_rx_isr, self);
  uart_irq_rx_enable(dev);

  return 0;
}

CommunicationInterface *ZephyrUartBin_viewAsCommunicationInterface(ZephyrUartBin *self) {
  return &self->communicationInterfaceView;
}
