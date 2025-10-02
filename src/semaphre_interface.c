#include "semaphore_interface.h"

bool SemaphoreInterface_take(SemaphoreInterface *self) {
  return self->take(self->instance);
}

void SemaphoreInterface_give(SemaphoreInterface *self) {
  return self->give(self->instance);
}

void SemaphoreInterface_reset(SemaphoreInterface *self) {
  return self->reset(self->instance);
}
