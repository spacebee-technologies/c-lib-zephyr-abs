#include "zephyr_udp.h"

#include "logger.h"

//******************************************************************************
// Interface implementations prototypes
//******************************************************************************
static uint8_t ZephyrUdp_send(void *self, const uint8_t *buffer, size_t bufferSize);
static uint8_t ZephyrUdp_receive(void *self, uint8_t *buffer, size_t bufferSize,
                                 size_t *receivedSize);

//******************************************************************************
// Private methods
//******************************************************************************
static void ZephyrUdp_initializeLanderAddress(ZephyrUdp *self,
                                              const char *landerAddress,
                                              uint16_t destinationPort) {
  self->landerAddress.sin_family = AF_INET;
  net_addr_pton(AF_INET, landerAddress, &self->landerAddress.sin_addr);
  self->landerAddress.sin_port = htons(destinationPort);
}

static void ZephyrUdp_initializeSocket(ZephyrUdp *self) {
  self->socketFileDescriptor = zsock_socket(AF_INET, SOCK_DGRAM, 0);
  if (self->socketFileDescriptor < 0) { LOG_ERROR("Error: socket: %d\n", errno); }
}

static void ZephyrUdp_initializeInterface(ZephyrUdp *self) {
  self->communicationInterfaceView.instance = self;
  self->communicationInterfaceView.send = ZephyrUdp_send;
  self->communicationInterfaceView.receive = ZephyrUdp_receive;
}

//******************************************************************************
// Interface implementations
//******************************************************************************
static uint8_t ZephyrUdp_send(void *self, const uint8_t *buffer, size_t bufferSize) {
  ZephyrUdp *_self = (ZephyrUdp *)self;
  zsock_sendto(_self->socketFileDescriptor, buffer, bufferSize, 0,
               (struct sockaddr *)&_self->landerAddress,
               sizeof(_self->landerAddress));
  return 0;
}

static uint8_t ZephyrUdp_receive(void *self, uint8_t *buffer, size_t bufferSize,
                                 size_t *receivedSize) {
  ZephyrUdp *_self = (ZephyrUdp *)self;
  socklen_t client_length = sizeof(_self->landerAddress);
  size_t returnValue = zsock_recvfrom(_self->socketFileDescriptor, buffer, bufferSize,
                                      ZSOCK_MSG_WAITALL, (struct sockaddr *)&_self->landerAddress,
                                      &client_length);
  if (returnValue != -1) {
    *receivedSize = returnValue;
    return 1;
  }
  return 0;
}

//******************************************************************************
// Public methods
//******************************************************************************
void ZephyrUdp_create(ZephyrUdp *self, const char *landerAddress, uint16_t landerPort) {
  ZephyrUdp_initializeInterface(self);
  ZephyrUdp_initializeSocket(self);
  ZephyrUdp_initializeLanderAddress(self, landerAddress, landerPort);
}

void ZephyrUdp_bind(ZephyrUdp *self, uint16_t port) {
  memset(&self->roverAddress, 0, sizeof(self->roverAddress));
  self->roverAddress.sin_family = AF_INET;
  self->roverAddress.sin_addr.s_addr = INADDR_ANY;
  self->roverAddress.sin_port = htons(port);
  int bindStatus = zsock_bind(self->socketFileDescriptor,
                              (struct sockaddr *)&self->roverAddress,
                              sizeof(self->roverAddress));
  if (bindStatus < 0) { LOG_ERROR("Error binding socket to port %d\n", port); }
}

CommunicationInterface *ZephyrUdp_viewAsCommunicationInterface(ZephyrUdp *self) {
  return &self->communicationInterfaceView;
}
