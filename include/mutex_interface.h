#ifndef MUTEX_INTERFACE_H_
#define MUTEX_INTERFACE_H_

#include <stdbool.h>

typedef struct MutexInterface {
  void *instance;
  bool (*acquire)(void *self);
  void (*release)(void *self);
} MutexInterface;

/**
 * @brief Acquire a mutex
 *
 * @param self A MutexInterface instance
 * @return true if the mutex was taken successfully
 * @return false if the mutex could not be taken
 */
bool MutexInterface_acquire(MutexInterface *self);

/**
 * @brief Release a mutex
 *
 * @param self A MutexInterface instance
 */
void MutexInterface_release(MutexInterface *self);

#endif  // MUTEX_INTERFACE_H_
