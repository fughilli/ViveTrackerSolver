/*
 * debug_serial.c
 *
 *  Created on: Jan 3, 2015
 *      Author: Kevin
 */

#include "driver_serial.h"

#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "utils/circular_buffer.h"
#include "driver_system.h"

#include <stdint.h>

typedef struct
{
    uint8_t rxbuffer_data[SERIAL_RXBUFFER_DEPTH],
            rxprebuffer_data[SERIAL_RXPREBUFFER_DEPTH],
            txbuffer_data[SERIAL_TXBUFFER_DEPTH],
            txprebuffer_data[SERIAL_TXPREBUFFER_DEPTH];
    circular_buffer_t txbuffer, rxbuffer, txprebuffer, rxprebuffer;
} Serial_module_buffers_t;

Serial_module_buffers_t uart_buffers[8];

bool uart_rx_locks[8] = { 0 };
bool uart_tx_locks[8] = { 0 };

const uint32_t uart_bases[8] =
{
UART0_BASE,
UART1_BASE,
UART2_BASE,
UART3_BASE,
UART4_BASE,
UART5_BASE,
UART6_BASE,
UART7_BASE, };

const uint32_t uart_peripherals[8] =
{
SYSCTL_PERIPH_UART0,
SYSCTL_PERIPH_UART1,
SYSCTL_PERIPH_UART2,
SYSCTL_PERIPH_UART3,
SYSCTL_PERIPH_UART4,
SYSCTL_PERIPH_UART5,
SYSCTL_PERIPH_UART6,
SYSCTL_PERIPH_UART7, };

const uint32_t gpio_peripherals[8] =
{
SYSCTL_PERIPH_GPIOA,
SYSCTL_PERIPH_GPIOB,
SYSCTL_PERIPH_GPIOD,
SYSCTL_PERIPH_GPIOC,
SYSCTL_PERIPH_GPIOC,
SYSCTL_PERIPH_GPIOE,
SYSCTL_PERIPH_GPIOD,
SYSCTL_PERIPH_GPIOE };

const uint32_t gpio_bases[8] =
{
GPIO_PORTA_BASE,
GPIO_PORTB_BASE,
GPIO_PORTD_BASE,
GPIO_PORTC_BASE,
GPIO_PORTC_BASE,
GPIO_PORTE_BASE,
GPIO_PORTD_BASE,
GPIO_PORTE_BASE };

const uint32_t gpio_configs_rx[8] =
{ GPIO_PA0_U0RX, GPIO_PB0_U1RX, GPIO_PD6_U2RX, GPIO_PC6_U3RX, GPIO_PC4_U4RX,
  GPIO_PE4_U5RX, GPIO_PD4_U6RX, GPIO_PE0_U7RX };

const uint32_t gpio_configs_tx[8] =
{ GPIO_PA1_U0TX, GPIO_PB1_U1TX, GPIO_PD7_U2TX, GPIO_PC7_U3TX, GPIO_PC5_U4TX,
  GPIO_PE5_U5TX, GPIO_PD5_U6TX, GPIO_PE1_U7TX };

const uint32_t uart_gpio_pins[8] =
{
GPIO_PIN_0 | GPIO_PIN_1,
GPIO_PIN_0 | GPIO_PIN_1,
GPIO_PIN_6 | GPIO_PIN_7,
GPIO_PIN_6 | GPIO_PIN_7,
GPIO_PIN_4 | GPIO_PIN_5,
GPIO_PIN_4 | GPIO_PIN_5,
GPIO_PIN_4 | GPIO_PIN_5,
GPIO_PIN_0 | GPIO_PIN_1 };

const uint32_t uart_interrupts[8] =
{
    INT_UART0,
    INT_UART1,
    INT_UART2,
    INT_UART3,
    INT_UART4,
    INT_UART5,
    INT_UART6,
    INT_UART7
};

void Serial_ISR(Serial_module_e module)
{
    // Hang on to these pointers to avoid typing this shit a billion times
    circular_buffer_t* buffer = &uart_buffers[module].rxbuffer;
    circular_buffer_t* prebuffer = &uart_buffers[module].rxprebuffer;

    uint32_t intstatus = UARTIntStatus(uart_bases[module], true);

    //// If another piece of code is modifying the buffers, we have to push into
    //// the prebuffer
    //if(uart_rx_locks[module])
    //{
    //    circular_buffer_push(prebuffer, UARTCharGet(uart_bases[module]));
    //}
    //else
    //{
    //    // If there are characters in the prebuffer, push them into the main
    //    // buffer
    //    while(!circular_buffer_empty(prebuffer))
    //    {
    //        circular_buffer_push(buffer, circular_buffer_front(prebuffer));
    //        circular_buffer_pop(prebuffer);
    //    }

        while(UARTCharsAvail(uart_bases[module]))
        {
            // Push the new character into the buffer
            circular_buffer_push(buffer, UARTCharGet(uart_bases[module]));
        }
    //}

    UARTIntClear(uart_bases[module], intstatus);
}

void Serial_0_ISR() { Serial_ISR(Serial_module_0); }
void Serial_1_ISR() { Serial_ISR(Serial_module_1); }
void Serial_2_ISR() { Serial_ISR(Serial_module_2); }
void Serial_3_ISR() { Serial_ISR(Serial_module_3); }
void Serial_4_ISR() { Serial_ISR(Serial_module_4); }
void Serial_5_ISR() { Serial_ISR(Serial_module_5); }
void Serial_6_ISR() { Serial_ISR(Serial_module_6); }
void Serial_7_ISR() { Serial_ISR(Serial_module_7); }

void Serial_init(Serial_module_e module, uint32_t baud)
{
    circular_buffer_init(&uart_buffers[module].rxbuffer,
                         uart_buffers[module].rxbuffer_data,
                         SERIAL_RXBUFFER_DEPTH);
    circular_buffer_init(&uart_buffers[module].txbuffer,
                         uart_buffers[module].txbuffer_data,
                         SERIAL_TXBUFFER_DEPTH);
    circular_buffer_init(&uart_buffers[module].rxprebuffer,
                         uart_buffers[module].rxprebuffer_data,
                         SERIAL_RXPREBUFFER_DEPTH);
    circular_buffer_init(&uart_buffers[module].txprebuffer,
                         uart_buffers[module].txprebuffer_data,
                         SERIAL_TXPREBUFFER_DEPTH);

    uart_rx_locks[module] = false;
    uart_tx_locks[module] = false;

    SysCtlPeripheralEnable(uart_peripherals[module]);
    SysCtlPeripheralEnable(gpio_peripherals[module]);
    UARTClockSourceSet(uart_bases[module], UART_CLOCK_SYSTEM);
    UARTConfigSetExpClk(uart_bases[module], SysCtlClockGet(), baud,
    UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE);
    GPIOPinConfigure(gpio_configs_tx[module]);
    GPIOPinConfigure(gpio_configs_rx[module]);
    GPIOPinTypeUART(gpio_bases[module], uart_gpio_pins[module]);
    UARTFIFOEnable(uart_bases[module]);
    UARTFIFOLevelSet(uart_bases[module], UART_FIFO_TX1_8, UART_FIFO_RX1_8);

    UARTIntEnable(uart_bases[module], UART_INT_RX | UART_INT_RT);

//    UARTIntDisable(uart_bases[module], 0xFFFFFFFF);

//    UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
//    UARTIntRegister(UART0_BASE, Serial_ISR);
//    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
//    IntEnable(INT_UART0);

    UARTEnable(uart_bases[module]);
    IntEnable(uart_interrupts[module]);
}

void Serial_putc(Serial_module_e module, char c)
{
    UARTCharPut(uart_bases[module], c);
}

bool Serial_avail(Serial_module_e module)
{
    //return UARTCharsAvail(uart_bases[module]);
    //uart_rx_locks[module] = true;
    UARTIntDisable(uart_bases[module], UART_INT_RX);

    bool ret = !circular_buffer_empty(&uart_buffers[module].rxbuffer);

    //uart_rx_locks[module] = false;
    UARTIntEnable(uart_bases[module], UART_INT_RX);

    return ret;
}

int Serial_getc(Serial_module_e module)
{
    //if (!UARTCharsAvail(uart_bases[module]))
    //    return -1;
    //return (int) UARTCharGet(uart_bases[module]);
    //
    //uart_rx_locks[module] = true;
    UARTIntDisable(uart_bases[module], UART_INT_RX);

    if(!circular_buffer_empty(&uart_buffers[module].rxbuffer))
    {
        int temp = circular_buffer_front(&uart_buffers[module].rxbuffer);
        circular_buffer_pop(&uart_buffers[module].rxbuffer);

        //uart_rx_locks[module] = false;
        UARTIntEnable(uart_bases[module], UART_INT_RX);
        return temp;
    }

    //uart_rx_locks[module] = false;
    UARTIntEnable(uart_bases[module], UART_INT_RX);
    return -1;
}

void Serial_puts(Serial_module_e module, const char * s)
{
    while(*s)
    {
        UARTCharPut(uart_bases[module], *s++);
    }
}

void Serial_writebuf(Serial_module_e module, const uint8_t* buf, uint32_t len)
{
    uint32_t i;
    for (i = 0; i < len; i++)
    {
        UARTCharPut(uart_bases[module], buf[i]);
    }
}

/**
 * @brief Blocks until any buffered output is transmitted.
 *
 * @param module The serial module to flush.
 */
void Serial_flush(Serial_module_e module)
{
    while (UARTBusy(uart_bases[module]))
        ;
}

/**
 * @brief Clears the receive buffer.
 *
 * @param module The serial module to clear the receive buffer of.
 */
void Serial_clear(Serial_module_e module)
{
    while (Serial_avail(module))
        Serial_getc(module);
}

/**
 * @brief Waits on a serial device to receive some provided string within the
 * provided timeout. If the provided string is received, then Serial_expect()
 * evaluates to true. If the provided string is not received before timeout,
 * Serial_expect() evaluates to false.
 *
 * @param module The serial module to wait on.
 * @param str The string to wait for.
 * @timeout A The total time to spend waiting on characters before failing.
 * @return True if the string is received; false if not.
 */
bool Serial_expect(Serial_module_e module, const char* str, uint32_t timeout)
{
    /*
     * While we haven't reached the end of the string, and havent timed-out
     */
    while((*str) && timeout)
    {
        /*
         * Get the next available character and compare it against the head of
         * the string. If the characters don't match, return failure; otherwise,
         * check the next character.
         */
        if(Serial_avail(module))
        {
            char c = Serial_getc(module);
            if(c != (*str++))
                return false;
            else
                continue;
        }

        /*
         * No character was available; we'll wait 1 time unit, and then try
         * again. Decrement the timeout while we do this.
         */
        Sys_delay(1);
        timeout--;
    }

    /*
     * If we got here, it means one of two things; either we timed out, or we
     * hit the end of the string. If it's the former, then timeout should be
     * set. Otherwise, it is the latter. The former is a failure case, and the
     * latter is a success case. Therefore, return timeout (because it is falsey
     * when it has reached 0).
     */
    return timeout;
}
