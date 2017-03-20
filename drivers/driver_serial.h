/*
 * debug_serial.h
 *
 *  Created on: Jan 3, 2015
 *      Author: Kevin
 */

#ifndef DRIVER_SERIAL_H_
#define DRIVER_SERIAL_H_

#include <stdint.h>
#include <stdbool.h>

#define SERIAL_RXBUFFER_DEPTH (64)
#define SERIAL_TXBUFFER_DEPTH (64)
#define SERIAL_RXPREBUFFER_DEPTH (4)
#define SERIAL_TXPREBUFFER_DEPTH (4)

typedef enum
{
    Serial_module_0 = 0,
    Serial_module_1,
    Serial_module_2,
    Serial_module_3,
    Serial_module_4,
    Serial_module_5,
    Serial_module_6,
    Serial_module_7,
} Serial_module_e;

#define Serial_module_debug (Serial_module_0)
#ifdef __cplusplus
extern "C" {
#endif

void Serial_init(Serial_module_e module, uint32_t baud);
void Serial_putc(Serial_module_e module, char c);
int Serial_getc(Serial_module_e module);
void Serial_puts(Serial_module_e module, const char * s);
void Serial_writebuf(Serial_module_e module, const uint8_t* buf, uint32_t len);
void Serial_flush(Serial_module_e module);
bool Serial_avail(Serial_module_e module);
bool Serial_expect(Serial_module_e module, const char* str, uint32_t timeout);
void Serial_clear(Serial_module_e module);

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_SERIAL_H_ */
