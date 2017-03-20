/*
 * camera.h
 *
 *  Created on: Apr 17, 2015
 *      Author: Kevin
 */

#ifndef CAMERA_H_
#define CAMERA_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define CAMERA_SAMPLES (128)

#define CAMERA_NUMCAMERAS (2)

void camera_init();

typedef int16_t camera_sample_t;

typedef enum
{
    CAMERA_BUFFER_A = 0,
    CAMERA_BUFFER_B = 1
} camera_buffer_index_t;

typedef enum
{
    UNLOCKED = 0,
    LOCKED = 1
} camera_buffer_lock_t;

extern volatile camera_sample_t* camera_buffers[CAMERA_NUMCAMERAS];
extern volatile camera_buffer_lock_t camera_buffer_lock;

// Callback type:
// arg0 -> sample buffer
// arg1 -> number of samples
typedef void(*camera_callback_t)(camera_sample_t*, uint32_t);

void camera_registerCallback(camera_callback_t callback);

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_H_ */
