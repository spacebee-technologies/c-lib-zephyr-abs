#ifndef ZEPHYR_I2C_H_
#define ZEPHYR_I2C_H_

#include <stdint.h>

#include <zephyr/device.h>

#include "i2c_interface.h"

typedef struct ZephyrI2c {
  I2cInterface i2cInterfaceView;
  const struct device *i2cDev;
  uint32_t i2cCfg;
} ZephyrI2c;

/**
 * @brief Initializes a new handler for I2C communications
 *
 * @param self Uninitialized instance
 * @return uint8_t 0 on success, 1 if error
 */
uint8_t ZephyrI2c_new(ZephyrI2c *self, const struct device *dev);

/**
 * @brief View of Zephyr I2C handler as an I2C interface
 *
 * @param self Initialized instance
 * @return I2cInterface* pointer to the I2C interface
 */
I2cInterface *ZephyrI2c_viewAsI2cInterface(ZephyrI2c *self);

#endif // ZEPHYR_I2C_H_
