#ifndef ZEPHYR_SEMAPHORE_H_
#define ZEPHYR_SEMAPHORE_H_

#include <zephyr/kernel.h>

#include "semaphore_interface.h"

typedef struct ZephyrSemaphore {
  SemaphoreInterface semaphoreInterfaceView;
  struct k_sem semaphore;
} ZephyrSemaphore;

bool ZephyrSemaphore_new(ZephyrSemaphore *self);

SemaphoreInterface *ZephyrSemaphore_viewAsSemaphoreInterface(ZephyrSemaphore *self);

#endif  // ZEPHYR_SEMAPHORE_H_
