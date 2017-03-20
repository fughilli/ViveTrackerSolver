#include "driver_accel.h"

#include "inc/hw_i2c.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"
#include "drivers/i2c/Wire.h"

void accel_init()
{
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
    //I2CMasterInitExpClk(I2C2_BASE, SysCtlClockGet(), true);
    Wire.begin();
}

bool accel_poll(accel_t* ret)
{
    Wire.beginTransmission(0x18);
    Wire.write(0x20);
    Wire.write(0x27);
    Wire.endTransmission();

    Wire.beginTransmission(0x18);
    Wire.write(0xA8);
    Wire.endTransmission();
    Wire.requestFrom(0x18, 2);
    ret->x = Wire.read();
    ret->x <<= 8;
    ret->x |= Wire.read();
    ret->y = Wire.read();
    ret->y <<= 8;
    ret->y |= Wire.read();
    ret->z = Wire.read();
    ret->z <<= 8;
    ret->z |= Wire.read();
    Wire.endTransmission();
    //I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    //I2CMasterSlaveAddrSet(0, 0, 0);
    return true;
}
