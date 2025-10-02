#ifndef ZEPHYR_TIME_H_
#define ZEPHYR_TIME_H_

#include <stdint.h>

void ZephyrTime_sleepMilli(uint32_t time);
void ZephyrTime_busySleepMicro(uint32_t time);
uint32_t ZephyrTime_getUptimeMilli();

#endif  // ZEPHYR_TIME_H_
