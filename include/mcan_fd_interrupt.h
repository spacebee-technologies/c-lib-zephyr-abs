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
  APP_STATE_MCAN_RECEIVE,          // Estado Recibiendo mensaje
  APP_STATE_MCAN_TRANSMIT,         // Estado transmitiendo mensaje
  APP_STATE_MCAN_IDLE,             // Estado mcan inactivo
  APP_STATE_MCAN_XFER_SUCCESSFUL,  // Estado mensaje recibido o transmitido correctamente
  APP_STATE_MCAN_XFER_ERROR,       // Estado mensaje recibido o transmitido erroneamente
  APP_STATE_MCAN_USER_INPUT        // Esperando al usuario para enviar o recibir mensaje
} APP_STATES;

typedef struct McanFdInterrupt {
  uint32_t xferContext;
  APP_STATES state;
  uint32_t rxMessageId;
  uint8_t rxMessage[64];
  uint8_t rxMessageLength;
  const struct device *canDev;
  struct can_frame frame;
} McanFdInterrupt;

void McanFdInterrupt_new(McanFdInterrupt *self);
bool McanFdInterrupt_receive(McanFdInterrupt *self, uint32_t *rxMessageId, uint8_t *rxMessage, uint8_t *rxMessageLength);
void McanFdInterrupt_configure(McanFdInterrupt *self);
bool McanFdInterrupt_send(McanFdInterrupt *self, uint32_t messageId, uint8_t *message, uint8_t messageLength, MCAN_MODE mcanMode);
uint8_t McanFdInterrupt_getState(McanFdInterrupt *self);
void McanFdInterrupt_enable(McanFdInterrupt *self);

#endif  // MCAN_FD_INTERRUPT_H_
