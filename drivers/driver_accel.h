#ifndef DRIVER_ACCEL_H
#define DRIVER_ACCEL_H

//#ifdef __cplusplus
//extern "C" {
//#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint16_t x, y, z;
} accel_t;

void accel_init();
bool accel_poll(accel_t* ret);

//#ifdef __cplusplus
//}
//#endif

#endif // DRIVER_ACCEL_H
