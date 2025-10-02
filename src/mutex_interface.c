#include "mutex_interface.h"

bool MutexInterface_acquire(MutexInterface *self) {
  return self->acquire(self->instance);
}

void MutexInterface_release(MutexInterface *self) {
  return self->release(self->instance);
}
