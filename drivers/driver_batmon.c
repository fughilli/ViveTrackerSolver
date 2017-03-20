#include "driver_batmon.h"

#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"
#include "inc/hw_adc.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"

static uint16_t batmon_adc_buf;

/**
 * @brief Initializes the battery monitor. Enables the ADC and GPIOD modules,
 *        sets the reference.
 */
void batmon_init()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_1);

    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PWM0, 2);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH6 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 1);

    IntEnable(INT_ADC0SS1);
    ADCIntEnable(ADC0_BASE, 1);

    batmon_adc_buf = 0x000;
}

void batmon_adc0_handler()
{
    uint32_t buf[4];
    if(ADCSequenceDataGet(ADC0_BASE, 1, buf))
    {
        batmon_adc_buf = buf[0];
    }

    ADCIntClear(ADC0_BASE, 1);
}

uint16_t batmon_get()
{
    // Value, in bits (3.3V/(2**12))
    uint32_t ret = batmon_adc_buf;

    return (uint16_t)ret;
    
    // Scale to mV
    ret = (ret * 3300 * 32)/(4096 * 10);

    return (uint16_t)ret;
}
