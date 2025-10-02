#include "zephyr_mutex.h"

//******************************************************************************
// Interface implementations
//******************************************************************************
static bool ZephyrMutex_acquire(void *self) {
  ZephyrMutex *_self = (ZephyrMutex *)self;
  if (k_mutex_lock(&_self->mutex, K_MSEC(100)) == 0) {
    return true;
  } else {
    return false;
  }
}

static void ZephyrMutex_release(void *self) {
  ZephyrMutex *_self = (ZephyrMutex *)self;
  k_mutex_unlock(&_self->mutex);
}

//******************************************************************************
// Private methods
//******************************************************************************
static void ZephyrMutex_initializeInterface(ZephyrMutex *self) {
  self->mutexInterfaceView.instance = self;
  self->mutexInterfaceView.acquire = ZephyrMutex_acquire;
  self->mutexInterfaceView.release = ZephyrMutex_release;
}

//******************************************************************************
// Public methods
//******************************************************************************
void ZephyrMutex_new(ZephyrMutex *self) {
  ZephyrMutex_initializeInterface(self);

  k_mutex_init(&self->mutex);
}

MutexInterface *ZephyrMutex_viewAsMutexInterface(ZephyrMutex *self) {
  return &self->mutexInterfaceView;
}
