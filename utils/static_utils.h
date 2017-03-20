#ifndef STATIC_UTILS_H
#define STATIC_UTILS_H

#include <stdint.h>

#ifdef __cplusplus

typedef uint32_t word_t;

struct Util
{
    template<class Type>
    static void call_init()
    {
        Type::init();
    }

    template<class FirstType, class SecondType, class ...RestTypes>
    static void call_init()
    {
        call_init<SecondType, RestTypes...>();
        call_init<FirstType>();
    }
};

/**
 * Field class.
 */
template<typename T, uint8_t offset, uint8_t width>
struct Field
{
    constexpr static T getMask()
    {
        return static_cast<T>((((word_t)1) << width) - 1);
    }

    const Field& operator=(const T& other)
    {
        setValue(other);
        return *this;
    }

    void setValue(const T& value)
    {
        constexpr T mask = getMask();
        m_value = value & mask;
    }

    T m_value;
};

// Consider making this constexpr
template<typename T, typename R, uint8_t offset, uint8_t width>
constexpr T _regMask(const T& reg, const Field<R, offset, width>& field)
{
    return (static_cast<T>(field.getMask()) << offset);
}

template<typename T, typename Head>
constexpr T regMask(const T& reg, const Head& field)
{
    return _regMask(reg, field);
}

template<typename T, typename Head, typename... Tail>
constexpr T regMask(const T& reg, const Head& field, const Tail&... fields)
{
    return _regMask(reg, field) | regMask(reg, fields...);
}

template<typename T, typename R, uint8_t offset, uint8_t width>
constexpr T _valueMask(const T& reg, const Field<R, offset, width>& field)
{
    return (static_cast<T>(field.m_value) << offset);
}

template<typename T, typename Head>
constexpr T valueMask(const T& reg, const Head& field)
{
    return _valueMask(reg, field);
}

template<typename T, typename Head, typename... Tail>
constexpr T valueMask(const T& reg, const Head& field, const Tail&... fields)
{
    return _valueMask(reg, field) | valueMask(reg, fields...);
}

template<typename T, typename... Fields>
T modify(const T& reg, const Fields&... fields)
{
    return ((reg & ~(regMask(reg, fields...))) | valueMask(reg, fields...));
}

template<typename T, typename R, uint8_t offset, uint8_t width>
R extract(const T& reg, const Field<R, offset, width>& field)
{
    return static_cast<R>((reg >> offset) & field.getMask());
}

template<typename T, typename... Fields>
T overwrite(const T& reg, const Fields&... fields)
{
    return valueMask(reg, fields...);
}

#endif

#endif
