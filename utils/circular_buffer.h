#ifndef CIRCULAR_BUFFER_H
#define CURCULAR_BUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    uint8_t* data;
    uint16_t head, tail;
    uint16_t len;
} circular_buffer_t;

void circular_buffer_init(circular_buffer_t* cb, uint8_t* data, uint16_t len);
void circular_buffer_push(circular_buffer_t* cb, uint8_t data);
bool circular_buffer_pop(circular_buffer_t* cb);
uint8_t circular_buffer_front(circular_buffer_t* cb);
bool circular_buffer_empty(circular_buffer_t* cb);

#ifdef __cplusplus
}
#endif

#endif
