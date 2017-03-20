#include "circular_buffer.h"

#define WRAP_FORWARD(_i_,_s_) ((((_i_)+1) == (_s_))?(0):((_i_)+1))
#define WRAP_BACKWARD(_i_,_s_) (((_i_) == 0)?((_s_)-1):((_i_)-1))

void circular_buffer_init(circular_buffer_t* cb, uint8_t* data, uint16_t len)
{
    cb->head = cb->tail = 0;
    cb->data = data;
    cb->len = len;
}

void circular_buffer_push(circular_buffer_t* cb, uint8_t data)
{
    if(cb->head == WRAP_BACKWARD(cb->tail, cb->len))
        cb->tail = WRAP_FORWARD(cb->tail, cb->len);

    cb->data[cb->head] = data;

    cb->head = WRAP_FORWARD(cb->head, cb->len);
}

bool circular_buffer_pop(circular_buffer_t* cb)
{
    if(cb->tail == cb->head)
        return false;

    cb->tail = WRAP_FORWARD(cb->tail, cb->len);

    return true;
}

uint8_t circular_buffer_front(circular_buffer_t* cb)
{
    return cb->data[cb->tail];
}

bool circular_buffer_empty(circular_buffer_t* cb)
{
    return (cb->head == cb->tail);
}
