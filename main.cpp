#include "VectorLib/Matrix.h"
#include "VectorLib/Vector.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "drivers/SPI/SPI.h"
#include "drivers/driver_serial.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_memmap.h"
#include "solver.hpp"
#include "drivers/driver_pin.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define DEVICE_SENSOR_COUNT 4

typedef Device<DEVICE_SENSOR_COUNT> Device4;
typedef Lighthouse<DEVICE_SENSOR_COUNT> Lighthouse4;
typedef Solver<DEVICE_SENSOR_COUNT, Device, Lighthouse> Solver4;


const Vector3d sensor_positions[] = {Vector3d(4.8,0,0.3),
                                     Vector3d(0,0,4.0),
                                     Vector3d(-2.1,4.1,0.3),
                                     Vector3d(-2.1,-4.1,0.3)};

Device4 current_device(sensor_positions);
Lighthouse4 lighthouse;

typedef Pin::DigitalPin<SYSCTL_PERIPH_GPIOB, GPIO_PORTB_BASE, 2> data_ready;
typedef Pin::DigitalPin<SYSCTL_PERIPH_GPIOB, GPIO_PORTB_BASE, 1> scan_dir;

typedef struct
{
    uint16_t angles[4];
} sensor_data_t;

typedef union
{
    sensor_data_t data;
    uint8_t raw[8];
} sensor_data_u;

int main(void)
{
    SysCtlClockSet(
            SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN
                    | SYSCTL_XTAL_16MHZ);

    Serial_init(Serial_module_debug, 115200);

    Serial_puts(Serial_module_debug, "[DEBUG] Hello world!\r\n");

    current_device.set_transform(Matrix4x4(Vector3d(0.1,0.1,0.1)));
    current_device.update_world_positions();

    SPI_begin(3);

    data_ready::init(); 

    bool finished_transaction = false;
    sensor_data_u sensor_data;
    fp_type h_meas[4], v_meas[4];
    bool h_data = false, v_data = false;
    fp_type* which_to_fill;
    bool which_scan = false;
    char print_buffer[128];

    while(true)
    {
        if(!finished_transaction &&
           data_ready::getState() == Pin::State_e::STATE_HIGH)
        {
            // Get the data from the FPGA
            for(uint32_t i = 0; i < 8; i++)
               sensor_data.raw[i] = SPI_transfer(3, 0); 

            // Transaction is finished... we'll wait until the next rising edge
            finished_transaction = true;

            // Figure out which scan this was.
            //bool which_scan = scan_dir::getState() == Pin::State_e::STATE_HIGH;

            // Fill the h_ or v_ floating point arrays.
            if(which_scan)
            {
                h_data = true;
                which_to_fill = h_meas;
            }
            else
            {
                v_data = true;
                which_to_fill = v_meas;
            }

            for(uint32_t i = 0; i < 4; i++)
            {
                which_to_fill[i] = sensor_data.data.angles[i]/26042.0;
            }

            snprintf(print_buffer, 128, "FCLR: %f %f %f %f\r\n", which_to_fill[0], which_to_fill[1], which_to_fill[2], which_to_fill[3]);
            Serial_puts(Serial_module_debug, print_buffer);
           
            if(h_data && v_data)
            { 
                h_data = v_data = false;

                lighthouse.rays_from_measurements(h_meas, v_meas);

                current_device.set_transform(Solver4::solve(current_device, lighthouse,
                                                            10).orthogonalize());
                current_device.update_world_positions();
            }

            which_scan = !which_scan;
        }
        
        if(finished_transaction &&
           data_ready::getState() == Pin::State_e::STATE_LOW)
        {
            finished_transaction = false;
        }


    }
}
