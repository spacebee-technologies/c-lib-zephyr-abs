#include "zephyr_time.h"

#include <zephyr/kernel.h>

void ZephyrTime_sleepMilli(uint32_t time) {
  k_msleep(time);
}

void ZephyrTime_busySleepMicro(uint32_t time) {
  k_busy_wait(time);
}

uint32_t ZephyrTime_getUptimeMilli() {
  return k_uptime_get_32();
}
