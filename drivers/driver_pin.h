#ifndef DRIVER_PIN_H
#define DRIVER_PIN_H

#ifdef __cplusplus

#include <driverlib/sysctl.h>
#include <registers/registers_gpio.h>

#include <stdint.h>

namespace Pin
{
    enum Mode_e
    {
        MODE_INPUT = 0,
        MODE_OUTPUT = 1
    };

    enum State_e
    {
        STATE_LOW = 0,
        STATE_HIGH = 1
    };

    enum PullMode_e
    {
        PULLMODE_NONE = 0,
        PULLMODE_UP = 1,
        PULLMODE_DOWN = 2
    };

    /**
     * @brief This is a Digital Pin template class.
     */
    template<uint32_t sysctl_periph, intptr_t gpio_base, uint32_t gpio_pin>
    class DigitalPin
    {
        public:

            static void init()
            {
                /*
                 * Enable the clock to the given peripheral.
                 *
                 * [340]
                 */
                SysCtlPeripheralEnable(sysctl_periph);

                /*
                 * Select the GPIO function for the given pin on the given port.
                 *
                 * [672, 10.5-R10]
                 */
                gpio_registers->AFSEL &= ~gpio_pin_mask;

                /*
                 * Select 2mA drive strength.
                 *
                 * [673, 10.5-R11]
                 */
                gpio_registers->DR2R |= gpio_pin_mask;

                /*
                 * Disable pull-ups and pull-downs.
                 *
                 * [677, 10.5-R15; 679, 10.5-R16]
                 */
                gpio_registers->PUR &= ~gpio_pin_mask;
                gpio_registers->PDR &= ~gpio_pin_mask;

                /*
                 * Disable slew rate control.
                 *
                 * [681, 10.5-R17]
                 */
                gpio_registers->SLR &= ~gpio_pin_mask;

                /*
                 * Enable the digital functions.
                 *
                 * [682, 10.5-R18]
                 */
                gpio_registers->DEN |= gpio_pin_mask;

                gpio_registers->PUR &= ~gpio_pin_mask;
                gpio_registers->PDR &= ~gpio_pin_mask;

                setMode(MODE_INPUT);
            }

            static void setMode(Mode_e mode)
            {
                switch(mode)
                {
                    case MODE_OUTPUT:
                        gpio_registers->DIR |= gpio_pin_mask;
                        break;

                    case MODE_INPUT:
                        gpio_registers->DIR &= ~gpio_pin_mask;
                        break;
                }
            }

            static Mode_e getMode()
            {
                if(gpio_registers->DIR & gpio_pin_mask)
                    return MODE_OUTPUT;

                return MODE_INPUT;
            }

            static void setState(State_e state)
            {
                switch(state)
                {
                    case STATE_LOW:
                        gpio_registers->DATA[gpio_pin_mask << 2] = 0;
                        break;

                    case STATE_HIGH:
                        gpio_registers->DATA[gpio_pin_mask << 2] = 0xFF;
                        break;
                }
            }

            static State_e getState()
            {
                if(gpio_registers->DATA[gpio_pin_mask << 2])
                    return STATE_HIGH;

                return STATE_LOW;
            }

            static PullMode_e getPullMode()
            {
                if(gpio_registers->PUR & gpio_pin_mask)
                    return PULLMODE_UP;
                if(gpio_registers->PDR & gpio_pin_mask)
                    return PULLMODE_DOWN;
                return PULLMODE_NONE;
            }

            static void setPullMode(PullMode_e pullmode)
            {
                switch(pullmode)
                {
                    case PULLMODE_UP:
                        gpio_registers->PUR |= gpio_pin_mask;
                        break;

                    case PULLMODE_DOWN:
                        gpio_registers->PDR |= gpio_pin_mask;
                        break;

                    default:
                        break;
                }
            }

        private:
            static constexpr uint32_t gpio_pin_mask = 1 << gpio_pin;
            constexpr static RegistersGPIO::Map* const gpio_registers =
                RegisterUtil::declareAt<RegistersGPIO::Map, gpio_base>();
    };

    template<typename T>
    class AnalogPin
    {
        public:
            static T getValue() { return static_cast<T>(0); }
        private:
    };
}

#endif

#endif
