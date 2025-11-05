#ifndef MCAN_FD_INTERRUPT_H_
#define MCAN_FD_INTERRUPT_H_

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>

typedef enum {
  MCAN_MODE_NORMAL,
  MCAN_MODE_FD_STANDARD,
  MCAN_MODE_FD_EXTENDED,
  MCAN_MODE_EXTENDED
} MCAN_MODE;

typedef enum {
  APP_STATE_MCAN_RECEIVE,
  APP_STATE_MCAN_TRANSMIT,
  APP_STATE_MCAN_IDLE,
  APP_STATE_MCAN_XFER_SUCCESSFUL,
  APP_STATE_MCAN_XFER_ERROR,
  APP_STATE_MCAN_USER_INPUT,
} APP_STATES;

typedef struct McanFdInterrupt {
  APP_STATES xferContext;
  APP_STATES state;
  uint32_t rxMessageId;
  uint8_t rxMessage[64];
  uint8_t rxMessageLength;
  const struct device *canDev;
  struct can_frame frame;
  can_mode_t mode;
} McanFdInterrupt;

void McanFdInterrupt_new(McanFdInterrupt *self, const struct device *dev, can_mode_t mode);
bool McanFdInterrupt_receive(McanFdInterrupt *self, uint32_t *rxMessageId, uint8_t *rxMessage, uint8_t *rxMessageLength);
void McanFdInterrupt_configure(McanFdInterrupt *self);
bool McanFdInterrupt_send(McanFdInterrupt *self, uint32_t messageId, uint8_t *message, uint8_t messageLength, MCAN_MODE mcanMode);
uint8_t McanFdInterrupt_getState(McanFdInterrupt *self);
void McanFdInterrupt_enable(McanFdInterrupt *self);

#endif  // MCAN_FD_INTERRUPT_H_
