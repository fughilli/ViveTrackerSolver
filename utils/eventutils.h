#ifndef EVENTUTILS_H
#define EVENTUTILS_H

#include <stdint.h>

namespace EventUtils
{
    struct EventAdapter
    {
        static void fire();
    };

    struct NullEventAdapter : EventAdapter
    {
        static void fire() {}
    };

    template <uint32_t period, class EventAdapter>
    struct PeriodicTimer
    {
        static uint32_t tick_count;

        static bool tick()
        {
            tick_count++;
            if(tick_count == period)
            {
                tick_count = 0;
                EventAdapter::fire();
                return true;
            }
            return false;
        }
    };

    template <uint32_t period, class EventAdapter>
        uint32_t PeriodicTimer<period, EventAdapter>::tick_count = 0;
}

#endif // EVENTUTILS_H
