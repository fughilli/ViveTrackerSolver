#ifndef NUMERIC_UTILS_H
#define NUMERIC_UTILS_H

namespace NumericUtil
{

    template<typename T>
    T map(T value, T inLow, T inHigh, T outLow, T outHigh)
    {
        return ((outHigh - outLow)*(value - inLow)/(inHigh - inLow)) + outLow;
    }

    template<typename T, T inLow, T inHigh, T outLow, T outHigh>
    T static_map(T value)
    {
        return ((outHigh - outLow)*(value - inLow)/(inHigh - inLow)) + outLow;
    }

    template<typename T>
    T clamp(T value, T minVal, T maxVal)
    {
        return (value > maxVal) ? maxVal : ((value < minVal) ? minVal : value);
    }

    template<typename T, T minVal, T maxVal>
    T static_clamp(T value)
    {
        return (value > maxVal) ? maxVal : ((value < minVal) ? minVal : value);
    }

}

#endif
