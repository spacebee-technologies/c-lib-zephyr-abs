#include "mcan_fd_interrupt.h"

#include "logger.h"

/**
 * @brief Callback called when a CAN frame is received that matches a filter
 *
 * @param dev Zephyr device structure for the CAN controller
 * @param frame Frame containing the received CAN message
 * @param userData Custom user data provided when setting up the filter
 */
void rx_irq_callback(const struct device *dev, struct can_frame *frame, void *userData) {
  McanFdInterrupt *self = (McanFdInterrupt *)userData;

  self->rxMessageLength = can_dlc_to_bytes(frame->dlc);
  self->rxMessageId = frame->id;

  LOG_DEBUG("New received CAN message: ID = 0x%x, Length = 0x%x", self->rxMessageId, self->rxMessageLength);

  for (uint8_t i = 0; i < self->rxMessageLength; i++) {
    self->rxMessage[i] = frame->data[i];
    LOG_DEBUG("CAN received data [%u]: 0x%x", i, self->rxMessage[i]);
  }

  k_sched_lock();
  self->xferContext = APP_STATE_MCAN_RECEIVE;
  self->state = APP_STATE_MCAN_XFER_SUCCESSFUL;
  k_sched_unlock();
}

/**
 * @brief Callback called when a CAN frame transmission is completed
 *
 * @param dev Zephyr device structure for the CAN controller
 * @param error Error code indicating the result of the transmission
 * @param userData Custom user data provided when setting up the filter
 */
void tx_irq_callback(const struct device *dev, int error, void *userData) {
  McanFdInterrupt *self = (McanFdInterrupt *)userData;

  k_sched_lock();
  if (error != 0) {
    self->state = APP_STATE_MCAN_XFER_ERROR;
    LOG_DEBUG("Error sending CAN message");
  } else {
    self->state = APP_STATE_MCAN_XFER_SUCCESSFUL;
  }
  k_sched_unlock();
}

void McanFdInterrupt_new(McanFdInterrupt *self, const struct device *dev, can_mode_t mode) {
  self->xferContext = APP_STATE_MCAN_RECEIVE;
  self->state = APP_STATE_MCAN_USER_INPUT;
  self->rxMessageId = 0;
  memset(self->rxMessage, 0, 64);
  self->rxMessageLength = 0;
  self->canDev = dev;
  self->frame.flags = 0;
  self->mode = mode;
}

/**
 * @brief Receive a CAN message if available
 *
 * @param self Initialized McanFdInterrupt instance
 * @param rxMessageId Variable to store the received CAN message ID
 * @param rxMessage Buffer to store the received CAN message data
 * @param rxMessageLength Variable to store the length of the received CAN message
 * @return true if message received successfully
 * @return false if no message was received
 */
bool McanFdInterrupt_receive(McanFdInterrupt *self, uint32_t *rxMessageId, uint8_t *rxMessage, uint8_t *rxMessageLength) {
  if (self->state == APP_STATE_MCAN_XFER_SUCCESSFUL && self->xferContext == APP_STATE_MCAN_RECEIVE) {
    k_sched_lock();
    *rxMessageId = self->rxMessageId;
    for (uint8_t i = 0; i < self->rxMessageLength; i++) {
      rxMessage[i] = self->rxMessage[i];
    }
    *rxMessageLength = self->rxMessageLength;
    k_sched_unlock();
    return true;
  } else {
    return false;
  }
}

/**
 * @brief Configure the MCAN FD peripheral
 *
 * @param self Initialized McanFdInterrupt instance
 */
void McanFdInterrupt_configure(McanFdInterrupt *self) {
  int ret;

  if (!device_is_ready(self->canDev)) {
    LOG_DEBUG("CAN: Device %s not ready", self->canDev->name);
  }

  can_stop(self->canDev);

  can_set_bitrate(self->canDev, 500000);  // 500000 bps nominal bitrate
  can_set_bitrate_data(self->canDev, 2000000);  // 2 Mbps data bitrate for FD

  ret = can_set_mode(self->canDev, self->mode);
  if (ret != 0) {
    LOG_DEBUG("Error setting CAN mode");
  }

  ret = can_start(self->canDev);
  if (ret != 0) {
    LOG_DEBUG("Error starting CAN controller");
  }

  // Filter for SDO requests (0x600 + node ID)
  const struct can_filter sdoRequests = {
    .id = 0x600,
    .mask = 0x780,  // matches 0x600–0x67F
    .flags = 0,
  };

  ret = can_add_rx_filter(self->canDev, &rx_irq_callback, self, &sdoRequests);

  if (ret == -ENOSPC) {
    LOG_DEBUG("Error, no filter available!");
  }

  // Filter for SDO responses (0x580 + node ID)
  const struct can_filter sdoResponses = {
    .id = 0x580,
    .mask = 0x780,  // matches 0x580–0x5FF
    .flags = 0,
  };

  ret = can_add_rx_filter(self->canDev, &rx_irq_callback, self, &sdoResponses);

  if (ret == -ENOSPC) {
    LOG_DEBUG("Error, no filter available!");
  }

  LOG_DEBUG("Finished CAN configuration");
}

/**
 * @brief Send a message over CAN bus
 *
 * @param self Initialized McanFdInterrupt instance
 * @param messageId ID of the CAN message to send
 * @param message Pointer to the CAN message data to send
 * @param messageLength Length of the CAN message data to send
 * @param mcanMode MCAN operation mode
 * @return true if message was sent successfully
 * @return false if message could not be sent
 */
bool McanFdInterrupt_send(McanFdInterrupt *self, uint32_t messageId, uint8_t *message, uint8_t messageLength, MCAN_MODE mcanMode) {
  if (self->state == APP_STATE_MCAN_USER_INPUT) {

    self->frame.flags = 0;
    if (mcanMode == MCAN_MODE_FD_STANDARD || mcanMode == MCAN_MODE_FD_EXTENDED) {
      self->frame.flags |= CAN_FRAME_FDF;  // TODO: Check if we can support also BRS
    }
    if (mcanMode == MCAN_MODE_FD_EXTENDED || mcanMode == MCAN_MODE_EXTENDED) {
      self->frame.flags |= CAN_FRAME_IDE;
    }

    self->frame.id = messageId;

    for (uint8_t i = 0; i < messageLength; i++) {
      self->frame.data[i] = message[i];
    }

    self->frame.dlc = can_bytes_to_dlc(messageLength);

    uint8_t res = can_send(self->canDev, &self->frame, K_FOREVER, tx_irq_callback, self);
    if (res != 0) {
      return false;
    }

    k_sched_lock();
    self->state = APP_STATE_MCAN_TRANSMIT;
    self->xferContext = APP_STATE_MCAN_TRANSMIT;
    k_sched_unlock();
    return true;
  } else {
    return false;
  }
}

/**
 * @brief Get the current state of the MCAN FD peripheral
 *
 * @param self Initialized McanFdInterrupt instance
 * @return uint8_t State of the MCAN FD peripheral:
 *                 0 = No transmission or reception in progress
 *                 1 = Message received successfully after calling McanFdInterrupt_receive()
 *                 2 = Message transmitted successfully after calling McanFdInterrupt_send()
 *                 3 = Error receiving message after calling McanFdInterrupt_receive()
 *                 4 = Error transmitting message after calling McanFdInterrupt_send()
 */
uint8_t McanFdInterrupt_getState(McanFdInterrupt *self) {
  switch (self->state) {
    case APP_STATE_MCAN_XFER_SUCCESSFUL:
        return (self->xferContext == APP_STATE_MCAN_RECEIVE) ? 1 : 2;
    case APP_STATE_MCAN_XFER_ERROR:
        return (self->xferContext == APP_STATE_MCAN_RECEIVE) ? 3 : 4;
    default:
        return 0;
  }
}

/**
 * @brief Enable the MCAN FD peripheral for new transmission or reception
 *
 * @param self Initialized McanFdInterrupt instance
 */
void McanFdInterrupt_enable(McanFdInterrupt *self) {
  self->state = APP_STATE_MCAN_USER_INPUT;
}
