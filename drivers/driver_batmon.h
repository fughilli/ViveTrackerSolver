#ifndef DRIVER_BATMON_H
#define DRIVER_BATMON_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void batmon_init();
uint16_t batmon_get();

#ifdef __cplusplus
}
#endif

#endif // DRIVER_BATMON_H
