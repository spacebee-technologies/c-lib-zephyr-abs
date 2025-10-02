#ifndef SEMAPHORE_INTERFACE_H_
#define SEMAPHORE_INTERFACE_H_

#include <stdbool.h>

typedef struct SemaphoreInterface {
  void *instance;
  bool (*take)(void *self);
  void (*give)(void *self);
  void (*reset)(void *self);
} SemaphoreInterface;

/**
 * @brief Take the semaphore
 *
 * @param self A semaphore interface
 * @return true if semaphore was taken successfully
 * @return false if could not take semaphore
 */
bool SemaphoreInterface_take(SemaphoreInterface *self);

/**
 * @brief Give the semaphore releasing the resource
 *
 * @param self A semaphore interface
 */
void SemaphoreInterface_give(SemaphoreInterface *self);

/**
 * @brief Reset the semaphore count
 *
 * @param self A semaphore interface
 */
void SemaphoreInterface_reset(SemaphoreInterface *self);

#endif  // SEMAPHORE_INTERFACE_H_
