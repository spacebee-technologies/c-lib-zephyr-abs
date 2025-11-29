#include "zephyr_gpio.h"

#include "logger.h"

//******************************************************************************
// Interface implementations
//******************************************************************************
static bool ZephyrGpio_getPinState(void *self) {
  ZephyrGpio *_self = (ZephyrGpio *)self;
  return gpio_pin_get_dt(_self->gpio);
}

static void ZephyrGpio_setPinState(void *self, uint8_t state) {
  ZephyrGpio *_self = (ZephyrGpio *)self;
  gpio_pin_set_dt(_self->gpio, state);
}

//******************************************************************************
// Private methods
//******************************************************************************
static void ZephyrGpio_initializeInterface(ZephyrGpio *self) {
  self->gpioInterfaceView.instance = self;
  self->gpioInterfaceView.getPinState = ZephyrGpio_getPinState;
  self->gpioInterfaceView.setPinState = ZephyrGpio_setPinState;
}

//******************************************************************************
// Public methods
//******************************************************************************
void ZephyrGpio_new(ZephyrGpio *self, const struct gpio_dt_spec *gpio, gpio_flags_t flags) {
  ZephyrGpio_initializeInterface(self);

  self->gpio = gpio;

  if (!device_is_ready(self->gpio->port)) {
    LOG_ERROR("GPIO port not ready");
  }

  int ret = gpio_pin_configure_dt(self->gpio, flags);

  if (ret != 0) {
    LOG_ERROR("Failed to configure GPIO, error: %d", ret);
    return;
  }
}

GpioInterface *ZephyrGpio_viewAsGpioInterface(ZephyrGpio *self) {
  return &self->gpioInterfaceView;
}
