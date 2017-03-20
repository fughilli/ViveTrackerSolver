#include "driver_system.h"
#include "driverlib/sysctl.h"

void Sys_delay(uint32_t ms)
{
    uint32_t cfreq = SysCtlClockGet();
    SysCtlDelay((cfreq / 1000) * ms / 3);
}
