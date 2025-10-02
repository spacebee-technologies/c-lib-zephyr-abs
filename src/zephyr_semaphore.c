#include "zephyr_semaphore.h"

//******************************************************************************
// Interface implementations
//******************************************************************************
static bool ZephyrSemaphore_take(void *self) {
  ZephyrSemaphore *_self = (ZephyrSemaphore *)self;
  if (k_sem_take(&_self->semaphore, K_FOREVER) == 0) { return true; }
  return false;
}

static void ZephyrSemaphore_give(void *self) {
  ZephyrSemaphore *_self = (ZephyrSemaphore *)self;
  k_sem_give(&_self->semaphore);
}

static void ZephyrSemaphore_reset(void *self) {
  ZephyrSemaphore *_self = (ZephyrSemaphore *)self;
  k_sem_reset(&_self->semaphore);
}

//******************************************************************************
// Private methods
//******************************************************************************
static void ZephyrSemaphore_initializeInterface(ZephyrSemaphore *self) {
  self->semaphoreInterfaceView.instance = self;
  self->semaphoreInterfaceView.take = ZephyrSemaphore_take;
  self->semaphoreInterfaceView.give = ZephyrSemaphore_give;
  self->semaphoreInterfaceView.reset = ZephyrSemaphore_reset;
}

//******************************************************************************
// Public methods
//******************************************************************************
bool ZephyrSemaphore_new(ZephyrSemaphore *self) {
  ZephyrSemaphore_initializeInterface(self);

  if (k_sem_init(&self->semaphore, 0, K_SEM_MAX_LIMIT) != 0) {
    return false;
  }

  return true;
}

SemaphoreInterface *ZephyrSemaphore_viewAsSemaphoreInterface(ZephyrSemaphore *self) {
  return &self->semaphoreInterfaceView;
}
