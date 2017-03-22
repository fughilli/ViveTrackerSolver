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
#include "utils/fast_utils.h"
#include "utils/base64enc.hpp"
#include "adapters/debug_serial.hpp"
#include "utils/streamutils.h"
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

/*
 * Define the data structure and stream used to send the transform matrix and
 * ray set to the python script.
 */
typedef struct
{
    Matrix4x4 transform;
    Vector3d rays[DEVICE_SENSOR_COUNT];
} output_data_t;

/*
 * Encode the data in Base64 to overcome 7-th bit meddling on certain serial
 * channels (e.g., Bluetooth serial modules like the HC-05/HC-06).
 */
static output_data_t output_data;
typedef Base64::Base64StreamWriter<DebugSerialAdapter::DebugSerialWriter,
                                   Base64::Base64LookupMapper>
            SerialBase64Writer;

typedef StreamUtils::StaticStructSender<SerialBase64Writer, output_data_t,
                                        &output_data, sizeof(output_data_t)>
            TransformSender;

/*
 * Define the data structure received from the FPGA.
 */
typedef struct
{
    uint16_t angles[4];
} sensor_data_t;

typedef union
{
    sensor_data_t data;
    uint8_t raw[8];
} sensor_data_u;

/*
 * Define a PeriodicTimer to re-orthogonalize the transformation matrix every
 * 1000 data frames. This will prevent accumulating error from crippling the
 * solver.
 */
typedef EventUtils::PeriodicTimer<1000, EventUtils::NullEventAdapter>
    OrthogonalizeTimer;

int main(void)
{
    SysCtlClockSet(
            SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN
                    | SYSCTL_XTAL_16MHZ);

    Serial_init(Serial_module_debug, 460800);

    Serial_puts(Serial_module_debug, "[DEBUG] Hello world!\r\n");

    current_device.set_transform(Matrix4x4(Vector3d::i * 10));
    current_device.update_world_positions();

    SPI_begin(3);

    data_ready::init();
    scan_dir::init();

    data_ready::setMode(Pin::Mode_e::MODE_INPUT);
    scan_dir::setMode(Pin::Mode_e::MODE_INPUT);

    bool finished_transaction = false;
    sensor_data_u sensor_data;
    fp_type h_meas[4], v_meas[4];
    bool h_data = false, v_data = false;
    bool flipped = false;
    fp_type* which_to_fill;
    char print_buffer[128];
    float accum_error = 10000.0;

    while(true)
    {
        if(!finished_transaction &&
           data_ready::getState() == Pin::State_e::STATE_HIGH)
        {
            // Figure out which scan this was.
            bool which_scan = (scan_dir::getState() ==
                               Pin::State_e::STATE_HIGH);

            if(flipped)
                which_scan = !which_scan;

            // Get the data from the FPGA
            for(uint32_t i = 0; i < 8; i++)
               sensor_data.raw[i] = SPI_transfer(3, 0);

            // Transaction is finished... we'll wait until the next rising edge
            finished_transaction = true;

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

                /*
                 * GROSS HACK WORKAROUND:
                 * Ignore bad frames by discarding timer counts that are
                 * out-of-range.
                 */
                if(which_to_fill[i] > 1)
                {
                    if(which_scan)
                    {
                        h_data = false;
                    }
                    else
                    {
                        v_data = false;
                    }
                }
            }

            if(h_data && v_data)
            {
                h_data = v_data = false;

                /*
                 * Compute the target rays from the captured data.
                 */
                lighthouse.rays_from_measurements(h_meas, v_meas);

                /*
                 * Solve for the device transform by performing 10 solver
                 * iterations.
                 */
                Matrix4x4 test_transform = Solver4::solve(current_device,
                                                          lighthouse, 10);

                /*
                 * Re-orthogonalize the transform if necessary.
                 */
                if (OrthogonalizeTimer::tick())
                {
                    test_transform = test_transform.orthogonalize();
                }

                /*
                 * GROSS HACK WORKAROUND:
                 * Occasionally, there will be a bad data frame from the FPGA.
                 * This could be due to several reasons, and my suspicion falls
                 * on the SPI module in the Verilog implementation (or, at
                 * least, how it's connected to the rest of the design--
                 * something is probably causing a setup/hold violation). In any
                 * case, these bad data frames will possibly cause the solver
                 * to report that the device has moved behind the lighthouse,
                 * or cause the error to spike. By detecting these conditions,
                 * we can discard solutions that don't make sense, and hopefully
                 * avoid acting upon the bad frames.
                 */
                float error = Solver4::get_error(current_device, lighthouse);

                accum_error = accum_error * 0.9 + error * 0.1;

                if (test_transform.translation().x > 0 &&
                    error <= accum_error * 2)
                {
                    current_device.set_transform(test_transform);
                }

                current_device.update_world_positions();

                /*
                 * Copy the data into the send structure.
                 */
                output_data.transform = current_device.transform;
                memcpy(output_data.rays, lighthouse.rays, sizeof(lighthouse.rays));

                /*
                 * Send the data.
                 */
                TransformSender::send();
                Serial_puts(Serial_module_debug, "\r\n");

                /*
                 * Check if the user has sent us an 'f', for "flip", meaning
                 * that we got the phase lock off by one timing window, and the
                 * horizontal data is actually the vertical data and vice-versa.
                 */
                if(Serial_avail(Serial_module_debug))
                    if(Serial_getc(Serial_module_debug) == 'f')
                        flipped = !flipped;
            }
        }

        if(finished_transaction &&
           data_ready::getState() == Pin::State_e::STATE_LOW)
        {
            finished_transaction = false;
        }


    }
}
