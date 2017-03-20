/*
 * camera.c
 *
 *  Created on: Apr 17, 2015
 *      Author: Kevin
 */

#include "driver_camera.h"

#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/udma.h"
#include "driverlib/interrupt.h"

#include "inc/hw_gpio.h"
#include "inc/hw_pwm.h"
#include "inc/hw_adc.h"
#include "inc/hw_udma.h"
#include "inc/hw_ints.h"

#include "driverlib/pin_map.h"
#include "inc/hw_memmap.h"

#include "../driver_serial.h"

#include "global_settings.h"

uint8_t uDMAControlTable[1024] __attribute__ ((aligned(1024)));

#define CAMERA_CLOCK_DIV (32)

#define CAMERA_SAMPLE_PERIOD (64)

uint32_t camera_PWMClockFreq;

uint32_t intcounter;

volatile uint32_t testval;

volatile camera_sample_t camera_Aibuffers[2][CAMERA_SAMPLES];
volatile camera_sample_t camera_Bibuffers[2][CAMERA_SAMPLES];
volatile camera_sample_t *camera_buffers[CAMERA_NUMCAMERAS];
volatile uint32_t camera_ibuffer_selected;

typedef enum
{
    CAMERA_WRITER_A = 0,
    CAMERA_WRITER_B = 1
} camera_writer_t;

volatile camera_writer_t camera_writer;


//void camera_permute_readers_writers()
//{
//    switch (camera_writer)
//    {
//    case CAMERA_WRITER_A:
//        camera_buffers[CAMERA_BUFFER_A] =
//                camera_ibuffers[camera_ibuffer_selected];
//        camera_writer = CAMERA_WRITER_B;
//        break;
//    case CAMERA_WRITER_B:
//        camera_buffers[CAMERA_BUFFER_B] =
//                camera_ibuffers[camera_ibuffer_selected];
//        camera_writer = CAMERA_WRITER_A;
//        break;
//    }
//
//    camera_ibuffer_selected++;
//    if (camera_ibuffer_selected == (CAMERA_NUMCAMERAS + 1))
//        camera_ibuffer_selected = 0;
//
//}

bool chaPP = false, chbPP = false;

void PWMGen1Handler()
{
    PWMGenIntClear(PWM0_BASE, PWM_GEN_1, PWM_INT_CNT_LOAD);

//    camera_permute_readers_writers();

    while (ADCBusy(ADC0_BASE))
        ;
//    SysCtlPeripheralDisable(ADC0_BASE);
//    SysCtlPeripheralEnable(ADC0_BASE);
//    uDMAChannelDisable(UDMA_CHANNEL_ADC3);
    asm volatile (
                "dsb\n\t"
                "isb" :::
        );
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
//    ADCSequenceDisable(ADC0_BASE, 3);
    //ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PWM0, 0);
//    if (camera_writer == CAMERA_WRITER_B)
//    {
        ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PWM0, 0);
//        GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);
        ADCSequenceStepConfigure(ADC0_BASE, 3, 0,
                ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);
        ADCSequenceEnable(ADC0_BASE, 3);
//        ADCSequenceDMAEnable(ADC0_BASE, 3);
        uDMAChannelTransferSet(UDMA_CHANNEL_ADC3 | (chbPP ? UDMA_PRI_SELECT : UDMA_ALT_SELECT),
        UDMA_MODE_PINGPONG, (void*) (ADC0_BASE + ADC_O_SSFIFO3),
                (void*) camera_Bibuffers[camera_ibuffer_selected],
                CAMERA_SAMPLES);

        chbPP = !chbPP;
        // Read from physical camera B

//    }
//    if (camera_writer == CAMERA_WRITER_A)
//    {
        ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PWM0, 0);
//        GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
        ADCSequenceStepConfigure(ADC0_BASE, 2, 0,
                ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
        ADCSequenceEnable(ADC0_BASE, 2);
//        ADCSequenceDMAEnable(ADC0_BASE, 3);
        uDMAChannelTransferSet(UDMA_CHANNEL_ADC2 | (chaPP ? UDMA_PRI_SELECT : UDMA_ALT_SELECT),
        UDMA_MODE_PINGPONG, (void*) (ADC0_BASE + ADC_O_SSFIFO2),
                (void*) camera_Aibuffers[camera_ibuffer_selected],
                CAMERA_SAMPLES);

        chaPP = !chaPP;

        camera_ibuffer_selected++;
        if(camera_ibuffer_selected == 2)
            camera_ibuffer_selected = 0;

        camera_buffers[0] = camera_Aibuffers[camera_ibuffer_selected];
        camera_buffers[1] = camera_Bibuffers[camera_ibuffer_selected];
        // Read from physical camera A
//
//    }


//    uDMAChannelEnable(UDMA_CHANNEL_ADC3);

    asm volatile (
            "dsb\n\t"
            "isb" :::
    );
}

void camera_init()
{
    int i;

    Serial_puts(SERIAL_MODULE, "inside \"camera_init\"\r\n");
    // Calculate the PWM clock frequency
    camera_PWMClockFreq = SysCtlClockGet() / CAMERA_CLOCK_DIV;

    DEBUG_LINE("camera_init");

    // Enable the PWM peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    // Enable the GPIO port
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure PD0 as the PWM output for the drive motor
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);
    GPIOPinConfigure(GPIO_PB7_M0PWM1);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);
    GPIOPinConfigure(GPIO_PB4_M0PWM2);

    // Set the camera clock pulse period
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, CAMERA_SAMPLE_PERIOD);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, (CAMERA_SAMPLE_PERIOD / 2));

    // Set the camera enable pulse period
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, (CAMERA_SAMPLE_PERIOD * CAMERA_SAMPLES));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, ((CAMERA_SAMPLE_PERIOD / 2) * 2));

    DEBUG_LINE("camera_init");

    // Enable the PWM output
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT | PWM_OUT_2_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);

    PWMSyncTimeBase(PWM0_BASE, PWM_GEN_0_BIT | PWM_GEN_1_BIT);

    // Enable PWM trigger on zero count on Generator 0
    PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_0, PWM_TR_CNT_ZERO); // PWM_TR_CNT_ZERO/PWM_TR_CNT_LOAD

    // Trigger an interrupt on GEN1 load (to setup the uDMA transfer on a consistent time boundary)
    PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_1, PWM_INT_CNT_LOAD);

    DEBUG_LINE("camera_init");

    /********************************************
     *               ADC CONFIGURATION                *
     ********************************************
     */

    // Enable ADC0 module
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    //DEBUG_LINE("camera_init");

    //ADCClockConfigSet(ADC0_BASE, ADC_CC_CS_PIOSC | ADC_CLOCK_RATE_FULL, 1);

    DEBUG_LINE("camera_init");

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    // Camera Far
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
    // Camera Near
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);

    DEBUG_LINE("camera_init");

    // Configure and enable the ADC sequence; single sample
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PWM0, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);

    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PWM0, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 2);

    DEBUG_LINE("camera_init");

    ADCSequenceDMAEnable(ADC0_BASE, 3);
    ADCSequenceDMAEnable(ADC0_BASE, 2);

    DEBUG_LINE("camera_init");

    // Start writing into the first buffer
    camera_ibuffer_selected = 0;
    // Expose the other buffers
//    for(i = 0; i < CAMERA_NUMCAMERAS; i++)
//    {
//        camera_buffers[i] = camera_ibuffers[i];
//    }

    camera_buffers[0] = camera_Aibuffers[0];
    camera_buffers[1] = camera_Bibuffers[0];

    /********************************************
     *               uDMA CONFIGURATION            *
     ********************************************
     */

    // Enable the uDMA for normal and sleep operation
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UDMA);
    uDMAEnable();

    DEBUG_LINE("camera_init");

    // Set the position of the uDMA control table
    uDMAControlBaseSet(uDMAControlTable);

    // Put the uDMA table entry for ADC3 into a known state
    uDMAChannelAttributeDisable(UDMA_CHANNEL_ADC3,
            UDMA_ATTR_USEBURST |
            UDMA_ATTR_ALTSELECT |
            UDMA_ATTR_HIGH_PRIORITY |
            UDMA_ATTR_REQMASK);
    uDMAChannelAttributeDisable(UDMA_CHANNEL_ADC2,
            UDMA_ATTR_USEBURST |
            UDMA_ATTR_ALTSELECT |
            UDMA_ATTR_HIGH_PRIORITY |
            UDMA_ATTR_REQMASK);

    // Configure the primary and alternate uDMA channel structures
    uDMAChannelControlSet(UDMA_CHANNEL_ADC3 | UDMA_PRI_SELECT, UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);
    uDMAChannelControlSet(UDMA_CHANNEL_ADC3 | UDMA_ALT_SELECT, UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);

    uDMAChannelControlSet(UDMA_CHANNEL_ADC2 | UDMA_PRI_SELECT, UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);
    uDMAChannelControlSet(UDMA_CHANNEL_ADC2 | UDMA_ALT_SELECT, UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);

    // Configure the primary and alternate transfers for ping-pong operation
    uDMAChannelTransferSet(UDMA_CHANNEL_ADC3 | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG, (void*) (ADC0_BASE + ADC_O_SSFIFO3), (void*)camera_Bibuffers[1], CAMERA_SAMPLES);
    uDMAChannelTransferSet(UDMA_CHANNEL_ADC3 | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG, (void*) (ADC0_BASE + ADC_O_SSFIFO3), (void*)camera_Bibuffers[0], CAMERA_SAMPLES);

    uDMAChannelTransferSet(UDMA_CHANNEL_ADC2 | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG, (void*) (ADC0_BASE + ADC_O_SSFIFO2), (void*)camera_Aibuffers[1], CAMERA_SAMPLES);
    uDMAChannelTransferSet(UDMA_CHANNEL_ADC2 | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG, (void*) (ADC0_BASE + ADC_O_SSFIFO2), (void*)camera_Aibuffers[0], CAMERA_SAMPLES);

    DEBUG_LINE("camera_init");

    // Enable the ADC3 uDMA channel
    uDMAChannelEnable(UDMA_CHANNEL_ADC3);
    uDMAChannelEnable(UDMA_CHANNEL_ADC2);

    // Enable interrupts
    // IntEnable(INT_ADC0SS3);
    // ADCIntEnableEx(ADC0_BASE, ADC_INT_DMA_SS3);

    IntEnable(INT_PWM0_1);
    PWMIntEnable(PWM0_BASE, PWM_INT_GEN_1);


    DEBUG_LINE("camera_init");

}


void camera_registerCallback(camera_callback_t callback)
{

}
