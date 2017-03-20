#ifndef DRIVER_BT_H
#define DRIVER_BT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "driver_serial.h"

typedef enum
{
    BT_BAUD_1200 = 0, // AT+BAUD1
    BT_BAUD_2400, // AT+BAUD2
    BT_BAUD_4800, // AT+BAUD3
    BT_BAUD_9600, // AT+BAUD4
    BT_BAUD_19200, // AT+BAUD5
    BT_BAUD_38400, // AT+BAUD6
    BT_BAUD_57600, // AT+BAUD7
    BT_BAUD_115200, // AT+BAUD8
    BT_BAUD_230400, // AT+BAUD9
    BT_BAUD_460800, // AT+BAUDA
    BT_BAUD_921600, // AT+BAUDB
    BT_BAUD_1382400, // AT+BAUDC
} bt_baud_rate_t;

typedef struct
{
    bt_baud_rate_t baud;
    Serial_module_e sermodule;
} bt_module_t;

bool bt_init(bt_module_t* btmodule, bt_baud_rate_t baud, Serial_module_e sermodule);
bool bt_setname(bt_module_t* btmodule, const char* name);
bool bt_setpin(bt_module_t* btmodule, const char* pin);

#ifdef __cplusplus
}
#endif

#endif // DRIVER_BT_H
