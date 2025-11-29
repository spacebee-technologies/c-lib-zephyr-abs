#ifndef ZEPHYR_GPIO_H_
#define ZEPHYR_GPIO_H_

#include <zephyr/drivers/gpio.h>

#include "gpio_interface.h"

typedef struct ZephyrGpio {
  GpioInterface gpioInterfaceView;
  const struct gpio_dt_spec *gpio;
} ZephyrGpio;

/**
 * @brief Initializes a new handler for a GPIO
 *
 * @param self Uninitialized instance
 */
void ZephyrGpio_new(ZephyrGpio *self, const struct gpio_dt_spec *gpio, gpio_flags_t flags);

/**
 * @brief View of Zephyr GPIO handler as an GPIO interface
 *
 * @param self Initialized instance
 * @return GpioInterface* pointer to the GPIO interface
 */
GpioInterface *ZephyrGpio_viewAsGpioInterface(ZephyrGpio *self);

#endif  // ZEPHYR_GPIO_H_
