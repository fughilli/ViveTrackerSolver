#ifndef DEBUG_SERIAL_ADAPTER
#define DEBUG_SERIAL_ADAPTER

#include "../drivers/driver_serial.h"

namespace DebugSerialAdapter
{

struct DebugSerialWriter
{
    static void begin() {}
    static void end() {}
    static void write(char c) { Serial_putc(Serial_module_debug, c); }
};

}

#endif // DEBUG_SERIAL_ADAPTER
