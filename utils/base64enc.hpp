#ifndef _BASE_64_ENC_H_
#define _BASE_64_ENC_H_

#include <stdint.h>
#include "sym_utils.h"

namespace Base64
{
    static constexpr char start_delim = '^', end_delim = '$', pad_char = '=';
    
    class Base64CompMapper
    {
    private:
        static constexpr char char_6263[2] = {'+', '/'};
    
    public:
    
        /**
         * @brief Looks up the base-64 character for a given 6-bit value.
         *
         * @param data_6bit A 6-bit value to lookup the base-64 character for.
         * @return The base-64 character that corresponds to the given 6-bit value.
         */
        static char b64lookup(uint8_t data_6bit)
        {
            data_6bit &= 0x3F;
            if(data_6bit < 26)
                return 'A'+data_6bit;
            if(data_6bit < 52)
                return 'a'+(data_6bit-26);
            if(data_6bit < 62)
                return '0'+(data_6bit-52);
        
            return char_6263[data_6bit-62];
        }
        
        /**
         * @brief Looks up the 6-bit value for a given base-64 character.
         *
         * @param b64char A base-64 character to lookup the 6-bit value for.
         * @return The 6-bit value that corresponds to the given base-64 character.
         */
        static uint8_t b64reverselookup(char b64char)
        {
            b64char &= 0x7F;
            if(b64char >= 'A' && b64char <= 'Z')
                return b64char - 'A';
            if(b64char >= 'a' && b64char <= 'z')
                return b64char - 'a' + 26;
            if(b64char >= '0' && b64char <= '9')
                return b64char - '0' + 52;
            if(b64char == '+')
                return 62;
            if(b64char == '/')
                return 63;
        
            return 0;
        }
    
    };
    
    class Base64LookupMapper
    {
    private:
        static constexpr char b64lookupstr[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        static constexpr uint8_t b64rlookuparray[] =
        {
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 62, 0, 0, 0, 63,
            52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 0, 0, 0, 0, 0, 0,
            0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
            15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 0, 0, 0, 0, 0,
            0, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
            41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 0, 0, 0, 0, 0
        };
    
    public:
    
        /**
         * @brief Looks up the base-64 character for a given 6-bit value.
         *
         * @param data_6bit A 6-bit value to lookup the base-64 character for.
         * @return The base-64 character that corresponds to the given 6-bit value.
         */
        static char b64lookup(uint8_t data_6bit)
        {
            return b64lookupstr[data_6bit & 0x3F];
        }
        
        /**
         * @brief Looks up the 6-bit value for a given base-64 character.
         *
         * @param b64char A base-64 character to lookup the 6-bit value for.
         * @return The 6-bit value that corresponds to the given base-64 character.
         */
        static uint8_t b64reverselookup(char b64char)
        {
            return b64rlookuparray[b64char & 0x7F];
        }
    
    };
    
    constexpr char Base64LookupMapper::b64lookupstr[];
    constexpr uint8_t Base64LookupMapper::b64rlookuparray[];
    
    template<class UWStreamAdapter, class _Base64LookupMapper>
    class Base64StreamWriter
    {
    private:
        static SymUtils::ConstBitPipe<8, 6> _bp;

    public:
        static void begin()
        {
            UWStreamAdapter::write(start_delim);
        }

        static void end()
        {
            bool firstchar = true;
            while(_bp.len != 0)
            {
                _bp.push(0);
                while(_bp.len >= 6)
                {
                    if(firstchar)
                    {
                        UWStreamAdapter::write(_Base64LookupMapper::b64lookup(_bp.pull()));
                    }
                    else
                    {
                        UWStreamAdapter::write(pad_char);
                        _bp.pull();
                    }
                    firstchar = false;
                }
            }
            UWStreamAdapter::write(end_delim);
        }

        static void write(uint8_t b)
        {
            _bp.push(b);
            while(_bp.len >= 6)
                UWStreamAdapter::write(_Base64LookupMapper::b64lookup(_bp.pull()));
        }
    };

    template<class UWStreamAdapter, class _Base64LookupMapper>
    SymUtils::ConstBitPipe<8, 6> Base64StreamWriter<UWStreamAdapter, _Base64LookupMapper>::_bp;
    
    /*
     * ORStreamAdapter::rxbegincallback();
     *  -- fired when start_delim is encountered in the incoming character
     *     stream.
     * ORStreamAdapter::rxendcallback();
     *  -- fired when end_delim is encountered in the incoming character stream.
     * ORStreamAdapter::rxcallback(byte data);
     *  -- fired when a new byte of data is available from the decoded character
     *     stream.
     */
    template<class ORStreamAdapter, class _Base64LookupMapper>
    class Base64StreamReader
    {
    private:
        static SymUtils::ConstBitPipe<6, 8> _bp;
    
    public:
        static void rxcallback(char c)
        { 
            if(c == start_delim)
            {
                _bp.reset();
                ORStreamAdapter::rxbegincallback();
            }
            else if(c == end_delim)
            {
                ORStreamAdapter::rxendcallback();
            }
            else
            {
                if(c == pad_char)
                {
                    //if(_bp.len)
                    //{
                    //    //_bp.push(0);
                    //    //ORStreamAdapter::rxcallback(_bp.pull());
                    //    _bp.reset();
                    //}
                }
                else
                {
                    _bp.push(_Base64LookupMapper::b64reverselookup(c));
                    if(_bp.len >= 8)
                        ORStreamAdapter::rxcallback(_bp.pull());
                }
            }
        }
    };

    template<class ORStreamAdapter, class _Base64LookupMapper>
    SymUtils::ConstBitPipe<6, 8> Base64StreamReader<ORStreamAdapter, _Base64LookupMapper>::_bp;
    
    template<class UStreamAdapter, class _Base64LookupMapper>
    class Base64StreamEncoder
    {
    private:
        uint32_t m_b64sr_r, m_b64sr_w;
        uint8_t m_b64sr_r_i, m_b64sr_w_i;
        bool hit_end;
    
    
    public:
        
        /**
         * @brief Constructs a new Base64StreamEncoder.
         */
        Base64StreamEncoder()
        {
            m_b64sr_w_i = 0;
            m_b64sr_w_i = 0;
            m_b64sr_w = 0;
            m_b64sr_w = 0;
            hit_end = false;
        }
        
        /**
         * @brief Encodes one data byte into base-64, and writes it to the underlying
         * stream by invoking the write callback. Could potentially generate two calls
         * to the write callback, depending on how much remaining data there is from
         * previous bytes.
         *
         * @param data The byte to write.
         */
        void write(uint8_t data)
        {
            m_b64sr_w <<= 8;
            m_b64sr_w |= data;
            m_b64sr_w_i += 2;
        
        
            UStreamAdapter::write(_Base64LookupMapper::b64lookup(m_b64sr_w >>
                                                                 m_b64sr_w_i));
        
            if(m_b64sr_w_i == 6)
            {
                UStreamAdapter::write(_Base64LookupMapper::b64lookup(m_b64sr_w));
                m_b64sr_w_i = 0;
            }
        }
        
        /**
         * @brief Encodes a byte buffer into base-64, and writes it to the underlying
         * stream by invoking the write callback. Internally, this method calls write()
         * repeatedly for every character in i_data.
         *
         * @param i_data The data buffer to write.
         * @param len The length of the data buffer to write.
         */
        void write(const uint8_t* i_data, uint32_t len)
        {
            UStreamAdapter::write(start_delim);
            for(uint32_t i = 0; i < len; i++)
                write(i_data[i]);
            flush();
            UStreamAdapter::write(end_delim);
        }
        
        /**
         * @brief Flushes any remaining data to the underlying stream. If there is not
         * a full character worth of data left, pad the remainder with zeros. Finally,
         * pad the entire stream length to a multiple of 4 with '='.
         */
        void flush()
        {
            if(m_b64sr_w_i)
            {
                UStreamAdapter::write(
                    _Base64LookupMapper::b64lookup(m_b64sr_w << (6 - m_b64sr_w_i)));
                for(uint8_t i = 1; i < (8-m_b64sr_w_i)/2; i++)
                {
                    UStreamAdapter::write('=');
                }
            }
        
            m_b64sr_w_i = 0;
        }
        
        /**
         * @brief Flushes the underlying stream's receive buffer of a frame.
         */
        void flushr()
        {
            m_b64sr_r_i = 0;
        
            if(available() == 0)
                return;
        
            while(UStreamAdapter::available() > 0)
            {
                if(UStreamAdapter::read() == '$')
                    break;
            }
        }
        
        /**
         * @brief Returns the available bytes in the underlying stream's receive buffer.
         * Calculated as a ratio of the available base-64 characters.
         *
         * @return The number of available bytes.
         */
        int available()
        {
            return (UStreamAdapter::available()*3)/4;
        }
        
        /**
         * @brief Reads a byte from the base-64 encoded stream. This method will make
         * calls to the read callback on the underlying stream to get enough base-64
         * characters to construct the next byte, and then return that byte.
         *
         * @return The next byte from the stream.
         */
        uint8_t read()
        {
            if(available() == 0)
                return 0;
        
            m_b64sr_r <<= 6;
            char rc = UStreamAdapter::read();
            if(rc == end_delim)
                return 0;
            m_b64sr_r |= _Base64LookupMapper::b64reverselookup(rc);
            if(m_b64sr_r_i == 0)
            {
                m_b64sr_r <<= 6;
                rc = UStreamAdapter::read();
                if(rc == end_delim)
                    return 0;
                m_b64sr_r |= _Base64LookupMapper::b64reverselookup(rc);
                m_b64sr_r_i = 6;
            }
            m_b64sr_r_i -= 2;
            return(m_b64sr_r >> m_b64sr_r_i) & 0xFF;
        }
        
        /**
         * @brief Reads a buffer of bytes from the base-64 encoded stream. This method
         * makes repeated calls to read() until the buffer is full, or a failure case
         * is encountered; in the case of failure, it returns 0; otherwise, the length
         * of the read data is returned.
         *
         * @param o_data A pointer to a buffer into which to read the data.
         * @param len The length of the buffer into which to read the data.
         * @return The length of the data read, or 0 on failure.
         */
        uint32_t read(uint8_t* o_data, uint32_t len)
        {
            if(available() == 0)
                return 0;
        
            bool fail = true;
            while(fail && UStreamAdapter::available() > 0)
            {
                if(UStreamAdapter::read() == '^')
                    fail = false;
            }
        
            if(fail)
                return 0;
        
            uint32_t i;
            for(i = 0; i < len && available() > 0; i++)
            {
                o_data[i] = read();
                if(hit_end)
                {
                    hit_end = false;
                    break;
                }
            }
            return i;
        }
        
        /**
         * @brief Decodes a base-64 encoded buffer. Returns the length of the data
         * written into the destination buffer.
         *
         * @param dest A pointer to the destination buffer.
         * @param destlen The length of the destination buffer.
         * @param src A pointer to the source buffer.
         * @param srclen The length of the source buffer.
         * @return The length of the data written into the destination buffer.
         */
        uint32_t decodeBuffer(uint8_t* dest,
            uint32_t destlen, const uint8_t* src, uint32_t srclen)
        {
            uint32_t srcidx = 0, destidx = 0;
        
            uint32_t partial = 0;
            uint32_t partial_idx = 0;
        
            while(srcidx < srclen)
            {
                partial <<= 6;
                char rc = src[srcidx++];
                if(rc == pad_char)
                    return destidx;
                partial |= _Base64LookupMapper::b64reverselookup(rc);
                if(partial_idx == 0)
                {
                    if(srcidx == srclen)
                        return 0;
                    partial <<= 6;
                    rc = src[srcidx++];
                    if(rc == pad_char)
                        return destidx;
                    partial |= _Base64LookupMapper::b64reverselookup(rc);
                    partial_idx = 6;
                }
                partial_idx -= 2;
                if(destidx == destlen)
                    return 0;
                dest[destidx++] = ((partial >> partial_idx) & 0xFF);
            }
        
            return destidx;
        }
        
        /**
         * @brief Encodes a byte buffer into a base-64 encoded buffer. Returns the
         * length of the data written into the destination buffer.
         *
         * @param dest A pointer to the destination buffer.
         * @param destlen The length of the destination buffer.
         * @param src A pointer to the source buffer.
         * @param srclen The length of the source buffer.
         * @return The length of the data written into the destination buffer.
         */
        uint32_t encodeBuffer(uint8_t* dest,
            uint32_t destlen, const uint8_t* src, uint32_t srclen)
        {
            uint32_t srcidx = 0, destidx = 0;
        
            while(srcidx < (srclen * 8))
            {
                if(destidx == destlen)
                    return 0;
        
                uint8_t offset = (srcidx % 8);
                uint8_t data = src[srcidx / 8];
                data >>= offset;
        
                if(offset > 2)
                {
                    data |= (src[(srcidx / 8) + 1] << (8-offset));
                }
        
                dest[destidx++] = _Base64LookupMapper::b64lookup(data);
        
                srcidx += 6;
            }
        
            return destidx;
        }
    
    };

}

#endif // _BASE_64_ENC_H_
