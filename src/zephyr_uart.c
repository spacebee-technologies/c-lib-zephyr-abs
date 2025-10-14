#include "zephyr_uart.h"

#include <string.h>

#include <zephyr/drivers/uart.h>

#include "logger.h"

//******************************************************************************
// Interface implementations
//******************************************************************************
static uint8_t ZephyrUart_send(void *self, const uint8_t *data, size_t length) {
  ZephyrUart *_self = (ZephyrUart *)self;
  if (!SemaphoreInterface_take(_self->sem)) { return 1; }
  for (int i = 0; i < length; i++) {
    uart_poll_out(_self->dev, data[i]);
  }
  SemaphoreInterface_give(_self->sem);
  return 0;
}

static uint8_t ZephyrUart_receive(void *self, uint8_t *buffer, size_t bufferSize, size_t *receivedSize) {
  ZephyrUart *_self = (ZephyrUart *)self;
  if (bufferSize < MSG_SIZE) { return 1; }
  char message[MSG_SIZE];
  int queueStatus = k_msgq_get(&_self->msgq, &message, K_NO_WAIT);
  if (queueStatus == 0) {
    *receivedSize = strlen(message);
    memcpy(buffer, message, *receivedSize);
    return 0;
  } else {
    return 1;
  }
}

//******************************************************************************
// Private methods
//******************************************************************************
static void ZephyrUart_serialCallback(const struct device *dev, void *uart) {
  ZephyrUart *_uart = (ZephyrUart *)uart;

  uint8_t c;

  if (!uart_irq_update(dev)) {
    return;
  }

  if (!uart_irq_rx_ready(dev)) {
    return;
  }

  // Read until FIFO empty
  while (uart_fifo_read(dev, &c, 1) == 1) {
    if ((c == '\n' || c == '\r') && _uart->rx_buf_pos > 0) {
      // Terminate string
      _uart->rx_buf[_uart->rx_buf_pos] = '\0';

      // If queue is full, message is silently dropped
      k_msgq_put(&_uart->msgq, &_uart->rx_buf, K_NO_WAIT);

      // Reset the buffer (it was copied to the msgq)
      _uart->rx_buf_pos = 0;
    } else if (_uart->rx_buf_pos < (sizeof(_uart->rx_buf) - 1)) {
      _uart->rx_buf[_uart->rx_buf_pos++] = c;
    }
    // Else: characters beyond buffer size are dropped
  }
}

static void ZephyrUart_initializeInterface(ZephyrUart *self) {
  self->communicationInterfaceView.instance = self;
  self->communicationInterfaceView.send = ZephyrUart_send;
  self->communicationInterfaceView.receive = ZephyrUart_receive;
}

//******************************************************************************
// Public methods
//******************************************************************************
uint8_t ZephyrUart_create(ZephyrUart *self, const struct device *dev, SemaphoreInterface *sem) {
  ZephyrUart_initializeInterface(self);
  self->dev = dev;
  self->rx_buf_pos = 0;
  k_msgq_init(&self->msgq, self->msgq_buffer, MSG_SIZE, MSGQ_ITEMS);
  self->sem = sem;
  return 0;
}

uint8_t ZephyrUart_configure(ZephyrUart *self) {
  if (!device_is_ready(self->dev)) {
    LOG_ERROR("UART device not found!");
    return 1;
  }

  int ret = 0;

  // Configure interrupt and callback to receive data
  ret = uart_irq_callback_user_data_set(self->dev, ZephyrUart_serialCallback, (void *)self);
  if (ret != 0) { return ret; }

  // Enable Rx interrupt
  uart_irq_rx_enable(self->dev);

  // Start semaphore unlocked
  if (ret != 0) {
    return ret;
  } else {
    SemaphoreInterface_give(self->sem);
  }

  return 0;
}

CommunicationInterface *ZephyrUart_viewAsCommunicationInterface(ZephyrUart *self) {
  return &self->communicationInterfaceView;
}
