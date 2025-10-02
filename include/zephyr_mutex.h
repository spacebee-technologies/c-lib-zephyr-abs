#ifndef ZEPHYR_MUTEX_H_
#define ZEPHYR_MUTEX_H_

#include <zephyr/kernel.h>

#include "mutex_interface.h"

typedef struct ZephyrMutex {
  MutexInterface mutexInterfaceView;
  struct k_mutex mutex;
} ZephyrMutex;

void ZephyrMutex_new(ZephyrMutex *self);

MutexInterface *ZephyrMutex_viewAsMutexInterface(ZephyrMutex *self);

#endif  // ZEPHYR_MUTEX_H_
