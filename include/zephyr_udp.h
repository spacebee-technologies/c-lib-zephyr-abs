#ifndef ZEPHYR_UDP_H_
#define ZEPHYR_UDP_H_

#include <zephyr/net/socket.h>

#include "communication_interface.h"

typedef struct ZephyrUdp {
  CommunicationInterface communicationInterfaceView;
  int socketFileDescriptor;
  struct sockaddr_in landerAddress;
  struct sockaddr_in roverAddress;
} ZephyrUdp;

void ZephyrUdp_create(ZephyrUdp *self, const char *landerAddress, uint16_t landerPort);

void ZephyrUdp_bind(ZephyrUdp *self, uint16_t roverPort);

CommunicationInterface *ZephyrUdp_viewAsCommunicationInterface(ZephyrUdp *self);

#endif  // ZEPHYR_UDP_H_
