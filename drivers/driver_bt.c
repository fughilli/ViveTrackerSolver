#include "driver_bt.h"
#include <stdlib.h>

#include "driver_system.h"

#define NUMBAUDS (12)

const uint32_t bt_bauds[] = {
    1200,
    2400,
    4800,
    9600,
    19200,
    38400,
    57600,
    115200,
    230400,
    460800,
    921600,
    1382400
};

const char bt_baud_chars[] = {
    '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C'
};

/**
 * @brief Initializes the Bluetooth module connected to the given serial port
 * with the given baud rate.
 *
 * @param btmodule A struct to be populated with the btmodule state, which can
 * be passed to subsequent calls to bt_*. If only initialization is necessary,
 * then this can be NULL.
 * @param baud The baud rate to initialize the module to.
 * @param sermodule The serial port that the Bluetooth module is connected to.
 */
bool bt_init(bt_module_t* btmodule, bt_baud_rate_t baud,
             Serial_module_e sermodule)
{
    int i = 0;

    /*
     * Populate the struct if it is given.
     */
    if(btmodule != NULL)
    {
        btmodule->baud = baud;
        btmodule->sermodule = sermodule;
    }

    /*
     * Check to see if the module is already at the desired baud. If so, we're
     * done.
     */
    Serial_init(sermodule, bt_bauds[baud]);
    Serial_puts(sermodule, "AT");
    if(Serial_expect(sermodule, "OK", 1000))
        return true;

    /*
     * Iterate through all possible bauds (except the one we want, because we
     * already checked it) and send the AT+BAUDx command to set the module to
     * the desired baud. If the module responds with OK, then we don't have to
     * check the rest.
     */
    for(i = 0; i < NUMBAUDS; i++)
    {
        if(i != (int)baud)
        {
            Serial_init(sermodule, bt_bauds[i]);
            Serial_puts(sermodule, "AT+BAUD");
            Serial_putc(sermodule, bt_baud_chars[baud]);

            if(Serial_expect(sermodule, "OK", 1000))
                break;
        }
    }

    /*
     * Wait a little while for anything else to be received from the module.
     */
    Sys_delay(1000);

    /*
     * Initialize the serial port to the desired baud, and do one final sanity
     * check. When the module is prompted with AT, it should respond with OK.
     * This indicates whether the initialization was successful.
     */
    Serial_init(sermodule, bt_bauds[baud]);

    /*
     * Clear any lingering characters.
     */
    Serial_clear(sermodule);

    /* sanity check */
    Serial_puts(sermodule, "AT");
    return Serial_expect(sermodule, "OK", 1000);
}

bool bt_setname(bt_module_t* btmodule, const char* name)
{
    Serial_clear(btmodule->sermodule);

    Serial_puts(btmodule->sermodule, "AT+NAME");
    Serial_puts(btmodule->sermodule, name);

    return Serial_expect(btmodule->sermodule, "OKsetname", 10000);
}

bool bt_setpin(bt_module_t* btmodule, const char* pin)
{
    Serial_clear(btmodule->sermodule);

    Serial_puts(btmodule->sermodule, "AT+PIN");
    Serial_puts(btmodule->sermodule, pin);
    
    return Serial_expect(btmodule->sermodule, "OKsetPIN", 10000);
}
