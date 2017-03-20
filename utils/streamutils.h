#ifndef STREAMUTILS_H
#define STREAMUTILS_H

#include "eventutils.h"

namespace StreamUtils
{
    template<class UWStreamAdapter>
    struct StructSender
    {
        static volatile uint8_t* startdatap;
        static volatile uint8_t* enddatap;

        template<typename _T>
        static void registerData(_T* data, uint32_t datalen)
        {
            startdatap = reinterpret_cast<volatile uint8_t*>(data);
            enddatap = startdatap + datalen;
        }

        static void send()
        {
            volatile uint8_t* _datap = startdatap;
            UWStreamAdapter::begin();
            while(_datap != enddatap)
                UWStreamAdapter::write(*_datap++);
            UWStreamAdapter::end();
        }
    };

    template<class UWStreamAdapter>
    volatile uint8_t* StructSender<UWStreamAdapter>::startdatap;
    template<class UWStreamAdapter>
    volatile uint8_t* StructSender<UWStreamAdapter>::enddatap;

    template<class EventAdapter = EventUtils::NullEventAdapter>
    struct StructReceiver
    {
        static volatile uint8_t* startdatap;
        static volatile uint8_t* enddatap;
        static volatile uint8_t* _datap;

        template<typename _T>
        static void registerData(_T* data, uint32_t datalen)
        {
            startdatap = reinterpret_cast<volatile uint8_t*>(data);
            enddatap = startdatap + datalen;
        }

        static void rxbegincallback()
        {
            _datap = startdatap;
        }

        static void rxendcallback()
        {
            EventAdapter::fire();
        }

        static void rxcallback(uint8_t data)
        {
            if(_datap == enddatap)
                return;
            *_datap++ = data;
        }
    };

    template<class EventAdapter>
    volatile uint8_t* StructReceiver<EventAdapter>::_datap;

    template<class UWStreamAdapter, typename T, T* data, uint32_t datalen>
    struct StaticStructSender
    {
        static volatile constexpr uint8_t* startdatap = reinterpret_cast<volatile uint8_t*>(data);
        static volatile constexpr uint8_t* enddatap = startdatap + datalen;

        static void send()
        {
            volatile uint8_t* _datap = startdatap;
            UWStreamAdapter::begin();
            while(_datap != enddatap)
                UWStreamAdapter::write(*_datap++);
            UWStreamAdapter::end();
        }
    };

    template<typename T, T* data, uint32_t datalen,
             class EventAdapter = EventUtils::NullEventAdapter>
    struct StaticStructReceiver
    {
        static volatile constexpr uint8_t* startdatap = reinterpret_cast<volatile uint8_t*>(data);
        static volatile constexpr uint8_t* enddatap = startdatap + datalen;
        static volatile uint8_t* _datap;
        static void rxbegincallback()
        {
            _datap = startdatap;
        }

        static void rxendcallback()
        {
            EventAdapter::fire();
        }

        static void rxcallback(uint8_t _data)
        {
            if(_datap == enddatap)
                return;
            *_datap++ = _data;
        }
    };

    template<typename T, T* data, uint32_t datalen, class EventAdapter>
    volatile uint8_t* StaticStructReceiver<T, data, datalen, EventAdapter>::_datap;
}

#endif // STREAMUTILS_H
