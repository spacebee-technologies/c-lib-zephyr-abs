#include "zephyr_i2c.h"

#include <zephyr/drivers/i2c.h>

//******************************************************************************
// Interface implementations
//******************************************************************************
static uint8_t ZephyrI2c_sendMessage(void *self, const uint8_t *txBuff, uint8_t nBytes, uint16_t address) {
  ZephyrI2c *_self = (ZephyrI2c *)self;
  return i2c_write(_self->i2cDev, txBuff, nBytes, address);
}

static uint8_t ZephyrI2c_readMessage(void *self, uint8_t *rxBuff, size_t bufferSize, uint16_t address) {
  ZephyrI2c *_self = (ZephyrI2c *)self;
  return i2c_read(_self->i2cDev, rxBuff, bufferSize, address);
}

static uint8_t ZephyrI2c_writeRead(void *self, uint16_t address, const uint8_t *txBuff,
                                   size_t txSize, uint8_t *rxBuff, size_t rxSize) {
  ZephyrI2c *_self = (ZephyrI2c *)self;
  return i2c_write_read(_self->i2cDev, address, txBuff, txSize, rxBuff, rxSize);
}

//******************************************************************************
// Private methods
//******************************************************************************
static void ZephyrI2c_initializeInterface(ZephyrI2c *self) {
  self->i2cInterfaceView.instance = self;
  self->i2cInterfaceView.sendMessage = ZephyrI2c_sendMessage;
  self->i2cInterfaceView.readMessage = ZephyrI2c_readMessage;
  self->i2cInterfaceView.writeRead = ZephyrI2c_writeRead;
}

//******************************************************************************
// Public methods
//******************************************************************************
uint8_t ZephyrI2c_new(ZephyrI2c *self, const struct device *dev) {
  ZephyrI2c_initializeInterface(self);

  self->i2cDev = dev;
  self->i2cCfg = I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER;

  if (!device_is_ready(self->i2cDev)) {
    return 1;
  }
  // Configure device
  if (i2c_configure(self->i2cDev, self->i2cCfg)) {
    return 1;
  }
  uint32_t i2cCfgTmp;
  // Verify the device was configured
  if (i2c_get_config(self->i2cDev, &i2cCfgTmp)) {
    return 1;
  }
  if (self->i2cCfg != i2cCfgTmp) {
    return 1;
  }
  return 0;
}

I2cInterface *ZephyrI2c_viewAsI2cInterface(ZephyrI2c *self) {
  return &self->i2cInterfaceView;
}
