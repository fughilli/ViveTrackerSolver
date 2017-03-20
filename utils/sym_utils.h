#ifndef SYM_UTILS_H
#define SYM_UTILS_H

#include <stdlib.h>
#include <stdint.h>

#define N_BITS_MASK(__bits__) ((1 << (__bits__)) - 1)

namespace SymUtils
{

typedef uint8_t symbol_t;

struct BitPipe
{
    uint64_t buffer;
    uint8_t len;

    BitPipe() : buffer(0), len(0) {}
    void push(uint64_t data, uint8_t width)
    {
        buffer = (buffer << width) | (data & N_BITS_MASK(width));
        len += width;
    }
    uint64_t pull(uint8_t width)
    {
        len -= width;
        return N_BITS_MASK(width) & (buffer >> len);
    }
    void reset()
    {
        buffer = 0;
        len = 0;
    }
};

template<uint8_t iwidth, uint8_t owidth>
struct ConstBitPipe
{
    uint64_t buffer;
    uint8_t len;

    ConstBitPipe() : buffer(0), len(0) {}
    void push(uint64_t data)
    {
        buffer = (buffer << iwidth) | (data & N_BITS_MASK(iwidth));
        len += iwidth;
    }
    uint64_t pull()
    {
        len -= owidth;
        return N_BITS_MASK(owidth) & (buffer >> len);
    }
    void reset()
    {
        buffer = 0;
        len = 0;
    }
};

}

#undef N_BITS_MASK

#endif // SYM_UTILS_H
