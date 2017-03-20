#ifndef EVENTUTILS_H
#define EVENTUTILS_H

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
}

#endif // EVENTUTILS_H
