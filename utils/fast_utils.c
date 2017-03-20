#include "fast_utils.h"
#include <stdarg.h>
#include <stdbool.h>

#define MAX_BASE (16)
const char __arb_base_digits[] = "0123456789ABCDEF"; //"0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz+/";

#define __digits_val_rev_lookup(_digit_) ((((_digit_) >= '0')&&((_digit_) <= '9'))?((_digit_)-'0'):((((_digit_) >= 'A')&&((_digit_) <= 'F'))?((_digit_)-('A'-10)):(-1)))

#if FAST_USE_ALLOC
static uint8_t alloc_mem[__ALLOC_MEM_SIZE];

#define hdr_payload(i)  ((HDR *) &alloc_mem[i])->payload
#define hdr_freeall(i)  ((HDR *) &alloc_mem[i])->freeall
#define hdr_succesr(i)  ((HDR *) &alloc_mem[i])->succesr
#define hdr_previus(i)  ((HDR *) &alloc_mem[i])->previus

#define sizehdr (sizeof(HDR))

typedef struct       // free block header/footer/linked list
{
    int  payload ;    // size of block (excluding headers)
    char freeall ;    // is this block allocated? 0=free/1=allocated
    int  succesr ;    // successor free block
    int  previus ;    // previous free block
} HDR;

static HDR anchor;

void init_heap()   // initialize the heap
{
    anchor.payload =  0 ;   // anchor
    anchor.freeall =  1 ;
    anchor.succesr =  0 ;
    anchor.previus = -1 ;

    hdr_payload(0) = __ALLOC_MEM_SIZE-2*sizehdr ;   // header
    hdr_freeall(0) = 0 ;
    hdr_succesr(0) = __ALLOC_MEM_SIZE ;
    hdr_previus(0) = -1 ;
    hdr_payload(__ALLOC_MEM_SIZE-sizehdr) = __ALLOC_MEM_SIZE-2*sizehdr ; // footer
    hdr_freeall(__ALLOC_MEM_SIZE-sizehdr) = 0 ;
}

void zero_header(int hi)
{
    int i;
    for(i = 0; i < sizehdr; i++)
        alloc_mem[hi + i] = 0;
}

void copy_header(int dest, int src)
{
    int i;
    for(i = 0; i < sizehdr; i++)
    	alloc_mem[dest + i] = alloc_mem[src + i];
}

#define prev_ftr(hdr) ((hdr) - sizehdr)
#define prev_hdr(hdr) ((hdr) - (sizehdr*2) - hdr_payload(prev_ftr(hdr)))
#define matching_hdr(ftr) ((ftr) - sizehdr - hdr_payload(ftr))
#define matching_ftr(hdr) ((hdr) + sizehdr + hdr_payload(hdr))
#define next_hdr(hdr) ((hdr) + (sizehdr*2) + hdr_payload(hdr))
#define next_ftr(hdr) (matching_ftr(next_hdr(hdr)))

void fast_free( void *buf )   // frees the block at address aa
{
    int headerpos = ((int)buf - (int)alloc_mem) - sizehdr;

    if(!hdr_freeall(headerpos))
        return;

    int nextheaderpos, nextfooterpos, prevheaderpos, prevfooterpos;

    // The position of the header/footer for aa
    int footerpos = matching_ftr(headerpos);

    int prevfree = 0, nextfree = 0;

    // The position for the header/footer of the block after aa
    if(footerpos != __ALLOC_MEM_SIZE - sizehdr)
    {
        nextheaderpos = next_hdr(headerpos);
        nextfooterpos = matching_ftr(nextheaderpos);
        nextfree = !hdr_freeall(nextheaderpos);
    }

    //int nowfree = __ALLOC_MEM_SIZE;

    // The position for the header/footer of the block before aa
    if(headerpos != 0)
    {
        //nowfree =
    	prevheaderpos = prev_hdr(headerpos);
        prevfooterpos = prev_ftr(headerpos);
        prevfree = !hdr_freeall(prevheaderpos);
    }

    if(nextfree && prevfree)
    {
        int prevprecessor = hdr_previus(prevheaderpos);
        int prevsuccessor = hdr_succesr(prevheaderpos);

        if(prevprecessor != -1)
        {
            hdr_succesr(prevprecessor) = prevsuccessor;
            hdr_succesr(matching_ftr(prevprecessor)) = prevsuccessor;
        }
        else
        {
            hdr_previus(prevsuccessor) = -1;
            anchor.succesr = prevsuccessor;

            if(anchor.succesr == __ALLOC_MEM_SIZE)
                anchor.succesr = 0;
        }

        if(prevsuccessor != __ALLOC_MEM_SIZE)
        {
            hdr_previus(prevsuccessor) = prevprecessor;
            hdr_previus(matching_ftr(prevsuccessor)) = prevprecessor;
        }

        int nextprecessor = hdr_previus(nextheaderpos);
        int nextsuccessor = hdr_succesr(nextheaderpos);

        if(nextprecessor != -1)
        {
            hdr_succesr(nextprecessor) = nextsuccessor;
            hdr_succesr(matching_ftr(nextprecessor)) = nextsuccessor;
        }
        else
        {
            hdr_previus(nextsuccessor) = -1;
            anchor.succesr = nextsuccessor;

            if(anchor.succesr == __ALLOC_MEM_SIZE)
                anchor.succesr = 0;
        }
        if(nextsuccessor != __ALLOC_MEM_SIZE)
        {
            hdr_previus(nextsuccessor) = nextprecessor;
            hdr_previus(matching_ftr(nextsuccessor)) = nextprecessor;
        }

        hdr_freeall(prevheaderpos) = 0;
        hdr_payload(prevheaderpos) = nextfooterpos - prevheaderpos - sizehdr;
        hdr_succesr(prevheaderpos) = anchor.succesr; //hdr_succesr(nextfooterpos);
        hdr_previus(prevheaderpos) = -1;

        hdr_previus(anchor.succesr) = prevheaderpos;
        hdr_previus(matching_ftr(anchor.succesr)) = prevheaderpos;
        anchor.succesr = prevheaderpos;

        if(anchor.succesr == __ALLOC_MEM_SIZE)
                anchor.succesr = 0;

        zero_header(nextheaderpos);
        zero_header(prevfooterpos);
        zero_header(headerpos);
        zero_header(footerpos);

        copy_header(nextfooterpos, prevheaderpos);
    }
    else if(nextfree)
    {
        hdr_freeall(headerpos) = 0;
        hdr_payload(headerpos) = nextfooterpos - headerpos - sizehdr;
        hdr_succesr(headerpos) = hdr_succesr(nextfooterpos);
        hdr_previus(headerpos) = hdr_previus(nextfooterpos);

        if(anchor.succesr == nextheaderpos)
            anchor.succesr = headerpos;

        if(hdr_succesr(headerpos) != __ALLOC_MEM_SIZE)
        {
            hdr_previus(hdr_succesr(headerpos)) = headerpos;
            hdr_previus(matching_ftr(hdr_succesr(headerpos))) = headerpos;
        }
        if(hdr_previus(headerpos) != -1)
        {
            hdr_succesr(hdr_previus(headerpos)) = headerpos;
            hdr_succesr(matching_ftr(hdr_previus(headerpos))) = headerpos;
        }

        copy_header(nextfooterpos, headerpos);

        zero_header(footerpos);
        zero_header(nextheaderpos);

        //nowfree = headerpos;
    }
    else if(prevfree)
    {
        hdr_freeall(prevheaderpos) = 0;
        hdr_payload(prevheaderpos) = footerpos - prevheaderpos - sizehdr;

        if(anchor.succesr == headerpos)
            anchor.succesr = prevheaderpos;

        zero_header(prevfooterpos);
        zero_header(headerpos);

        copy_header(footerpos, prevheaderpos);
    }
    else
    {
        hdr_freeall(headerpos) = 0;

        hdr_previus(headerpos) = -1;
        hdr_succesr(headerpos) = anchor.succesr;
        hdr_previus(anchor.succesr) = headerpos;

        copy_header(matching_ftr(headerpos), headerpos);
        copy_header(matching_ftr(anchor.succesr), anchor.succesr);

        anchor.succesr = headerpos;
        //nowfree = headerpos;
    }
}

void* fast_alloc( size_t sz )   // allocates a block of size int
{
    if(sz % sizehdr)
        sz = sz - (sz % sizehdr) + sizehdr; // Round up to nearest multiple of 16

    int curheaderpos = anchor.succesr; // Start at the first free block

    // Iterate through the linked list of free blocks
    for(; curheaderpos < __ALLOC_MEM_SIZE; curheaderpos = hdr_succesr(curheaderpos))
    {
        if(hdr_payload(curheaderpos) > (sz + sizehdr*2)) // room to split
        {
            // Compute some positions
            int next_footerpos = hdr_succesr(curheaderpos) - sizehdr;
            int footerpos = curheaderpos + sizehdr + sz;
            int next_headerpos = footerpos + sizehdr;
            int old_successor = hdr_succesr(curheaderpos);
            int old_precessor = hdr_previus(curheaderpos);

            hdr_freeall(curheaderpos) = 1;
            hdr_payload(curheaderpos) = sz;
            hdr_succesr(curheaderpos) = -1;
            hdr_succesr(curheaderpos) = -1;

            copy_header(footerpos, curheaderpos);

            hdr_freeall(next_headerpos) = 0;
            hdr_payload(next_headerpos) = next_footerpos - (next_headerpos + sizehdr);
            hdr_previus(next_headerpos) = old_precessor;
            hdr_succesr(next_headerpos) = old_successor;

            copy_header(next_footerpos, next_headerpos);

            if(anchor.succesr == curheaderpos)
            {
                anchor.succesr = next_headerpos;
            }

            return &alloc_mem[curheaderpos + sizehdr];

        }
        else if(hdr_payload(curheaderpos) >= sz) // no room to split, but still fits
        {
            hdr_freeall(curheaderpos) = 1;
            copy_header(hdr_succesr(curheaderpos) - sizehdr, curheaderpos);

//            int j;
//            for(j = curheaderpos; j < __ALLOC_MEM_SIZE; j = hdr_succesr(j))
//            {
//                if(!hdr_freeall(j))
//                {
//                    anchor.succesr = j;
//                    return curheaderpos + sizehdr;
//                }
//            }

            // If this was the first free block in the chain, move the anchor to the next one
            if(anchor.succesr == curheaderpos)
            {
                anchor.succesr = hdr_succesr(curheaderpos);
                hdr_previus(anchor.succesr) = -1;
            }
            else
            {
                if(hdr_succesr(curheaderpos) != __ALLOC_MEM_SIZE)
                {
                    hdr_previus(hdr_succesr(curheaderpos)) = hdr_previus(curheaderpos);
                    hdr_previus(matching_ftr(hdr_succesr(curheaderpos))) = hdr_previus(curheaderpos);
                }
                if(hdr_previus(curheaderpos) != -1)
                {
                    hdr_succesr(hdr_previus(curheaderpos)) = hdr_succesr(curheaderpos);
                    hdr_succesr(matching_ftr(hdr_previus(curheaderpos))) = hdr_succesr(curheaderpos);
                }
                else
                    anchor.succesr = hdr_succesr(curheaderpos);
            }

            hdr_succesr(curheaderpos) = -1;
            hdr_previus(curheaderpos) = -1;

            copy_header(matching_ftr(curheaderpos), curheaderpos);

            return &alloc_mem[curheaderpos + sizehdr];
        } // else, doesn't fit at all

        //prevheaderpos = curheaderpos;
    }

//    int old__ALLOC_MEM_SIZE = __ALLOC_MEM_SIZE;
//    __ALLOC_MEM_SIZE *= 2;
//    alloc_mem = (char*)realloc(alloc_mem, __ALLOC_MEM_SIZE);
//
//    int i;
//    for(i = old__ALLOC_MEM_SIZE; i < __ALLOC_MEM_SIZE; i++)
//        alloc_mem[i] = 0;
//
//    // If the block at the end of the alloc_mem is free
//    if(!hdr_freeall(old__ALLOC_MEM_SIZE - sizehdr))
//    {
//        // Expand the last block to fill up the new space
//        int oldendheaderpos = matching_hdr(old__ALLOC_MEM_SIZE - sizehdr);
//        zero_header(old__ALLOC_MEM_SIZE - sizehdr);
//
//        hdr_payload(oldendheaderpos) = __ALLOC_MEM_SIZE - oldendheaderpos - sizehdr*2;
//        hdr_succesr(oldendheaderpos) = __ALLOC_MEM_SIZE;
//        copy_header(__ALLOC_MEM_SIZE - sizehdr, oldendheaderpos);
//    }
//    else
//    {
//        // Add a new block
//        hdr_payload(old__ALLOC_MEM_SIZE) = old__ALLOC_MEM_SIZE - sizehdr*2;
//        hdr_freeall(old__ALLOC_MEM_SIZE) = 0;
//        hdr_succesr(old__ALLOC_MEM_SIZE) = __ALLOC_MEM_SIZE;
//        hdr_previus(old__ALLOC_MEM_SIZE) = prevheaderpos;
//
//        copy_header(__ALLOC_MEM_SIZE - sizehdr, old__ALLOC_MEM_SIZE);
//    }

    return NULL;

}
#endif

void fast_swap8(uint8_t* a, uint8_t* b)
{
	uint8_t temp = (*a);
	(*a) = (*b);
	(*b) = temp;
}

void fast_swap16(uint16_t* a, uint16_t* b)
{
	uint16_t temp = (*a);
	(*a) = (*b);
	(*b) = temp;
}

void fast_swap32(uint32_t* a, uint32_t* b)
{
	uint32_t temp = (*a);
	(*a) = (*b);
	(*b) = temp;
}

void fast_memcpy(void* dst, const void* src, size_t sz)
{
    if(dst == src || sz == 0)
        return;

    do
    {
        --sz;
        ((uint8_t*)dst)[sz] = ((uint8_t*)src)[sz];
    } while(sz);
}

void fast_memmove(void* dst, const void* src, size_t sz)
{
    size_t iter = 0;

    if(dst == src || sz == 0)
        return;

    if(dst > src)
        do
        {
            --sz;
            ((uint8_t*)dst)[sz] = ((uint8_t*)src)[sz];
        }
        while(sz);
    else
        do
        {
            ((uint8_t*)dst)[iter] = ((uint8_t*)src)[iter];
        }
        while(iter++ < sz);
}

void fast_memset(void* dst, uint8_t val, size_t sz)
{
    if(sz == 0)
        return;
    do
    {
        --sz;
        ((uint8_t*)dst)[sz] = val;
    } while(sz);
}

size_t fast_strlen(const char* str)
{
	size_t ret = 0;
	while(str[ret])
		ret++;
	return ret;
}

size_t fast_strnlen(const char* str, size_t maxlen)
{
	size_t ret = 0;
	while(str[ret] && (ret < maxlen))
		ret++;
	return ret;
}

int fast_memcmp(const void* ptr1, const void* ptr2, size_t sz)
{
	int8_t *a, *b;
	a = (int8_t*)ptr1;
	b = (int8_t*)ptr2;
	int ret = 0;

	while(sz && (ret == 0))
	{
		sz--;
		ret += (*a);
		ret -= (*b);
	}

	return ret;
}

size_t fast_strcpy(char* stra, const char* strb)
{
	size_t ret = fast_strlen(strb);
	fast_memcpy(stra, strb, ret+1);
	return ret;
}

size_t fast_strncpy(char* stra, const char* strb, size_t maxlen)
{
	size_t ret = fast_strnlen(strb, maxlen);
	if(ret > maxlen)
	{
		stra[maxlen-1] = 0;
		ret--;
	}
	if(ret < maxlen)
		ret++;
	fast_memcpy(stra, strb, ret);
	return ret;
}

int fast_strcmp(const char* stra, const char* strb)
{
	int ret = 0;
	while ((ret == 0) && ((*stra) || (*strb)))
	{
		ret += (*stra);
		ret -= (*strb);
		stra++;
		strb++;
	}
	return ret;
}

long fast_sntol(const char* str, size_t sz, unsigned int base, bool* succ)
{
	long ret = 0;

	size_t i = 0;
	bool sign = false;

	if (succ)
	{
		(*succ) = false;
	}

	if (base > MAX_BASE || sz == 0)
		return 0;

	if (sz >= 2 && str[0] == '-')
	{
		sign = true;
		i = 1;
	}

	for (; i < sz; i++)
	{
		int16_t charval = __digits_val_rev_lookup(str[i]);
		if (charval < 0 || charval >= base)
		{
			return 0;
		}

		ret *= base;
		ret += charval;
	}

	if (succ)
	{
		(*succ) = true;
	}

	if (sign)
		ret = -ret;
	return ret;
}

unsigned long fast_sntoul(const char* str, size_t sz, unsigned int base, bool* succ)
{
	unsigned long ret = 0;

	size_t i;

	if (succ)
	{
		(*succ) = false;
	}

	if (base > MAX_BASE || sz == 0)
		return 0;

	for (i = 0; i < sz; i++)
	{
		int16_t charval = __digits_val_rev_lookup(str[i]);
		if (charval < 0 || charval >= base)
		{
			return 0;
		}

		ret *= base;
		ret += charval;
	}

	if (succ)
	{
		(*succ) = true;
	}

	return ret;
}

float fast_sntof(const char* str, size_t sz, unsigned int base, bool* succ)
{
	float ret = 0;
	float retfpart = 0;

	size_t i = 0;
	bool sign = false;

	if (succ)
	{
		(*succ) = false;
	}

	if (base > MAX_BASE || sz == 0)
		return 0;

	if (sz >= 2 && str[0] == '-')
	{
		sign = true;
		i = 1;
	}

	for (; i < sz; i++)
	{
		if (str[i] == '.')
			break;
		int16_t charval = __digits_val_rev_lookup(str[i]);
		if (charval < 0 || charval >= base)
		{
			return 0;
		}

		ret *= base;
		ret += charval;
	}

	if (str[i])
	{

		int j = fast_strlen(&str[i + 1]);

		if ((j + i + 1) > sz)
			return 0;

		for (; j > 0; j--)
		{
			int16_t charval = __digits_val_rev_lookup(str[i + j]);
			if (charval < 0 || charval >= base)
			{
				return 0;
			}

			retfpart += charval;
			retfpart /= base;
		}

	}

	if (succ)
	{
		(*succ) = true;
	}

	ret += retfpart;

	if (sign)
		ret = -ret;
	return ret;
}

double fast_sntod(const char* str, size_t sz, unsigned int base, bool* succ)
{
	double ret = 0;
	double retfpart = 0;

	size_t i = 0;
	bool sign = false;

	if (succ)
	{
		(*succ) = false;
	}

	if (base > MAX_BASE || sz == 0)
		return 0;

	if (sz >= 2 && str[0] == '-')
	{
		sign = true;
		i = 1;
	}

	for (; i < sz; i++)
	{
		if (str[i] == '.')
			break;
		int16_t charval = __digits_val_rev_lookup(str[i]);
		if (charval < 0 || charval >= base)
		{
			return 0;
		}

		ret *= base;
		ret += charval;
	}

	if (str[i])
	{

		int j = fast_strlen(&str[i + 1]);

		if ((j + i + 1) > sz)
			return 0;

		for (; j > 0; j--)
		{
			int16_t charval = __digits_val_rev_lookup(str[i + j]);
			if (charval < 0 || charval >= base)
			{
				return 0;
			}

			retfpart += charval;
			retfpart /= base;
		}

	}

	if (succ)
	{
		(*succ) = true;
	}

	ret += retfpart;

	if (sign)
		ret = -ret;
	return ret;
}

bool fast_sntob(const char* str, size_t sz, bool* succ)
{
	if (sz < 4)
	{
		if (succ)
			(*succ) = false;
		return false;
	}

	if (fast_memcmp(str, "true", 4) == 0)
	{
		if (succ)
			(*succ) = true;
		return true;
	}

	if (fast_memcmp(str, "false", 5) == 0)
	{
		if (succ)
			(*succ) = true;
		return false;
	}

	if (succ)
		(*succ) = false;
	return false;
}

int fast_snfmtf(char* buf, size_t bufsiz, float f, size_t digits, unsigned long base)
{
	if (bufsiz == 0 || base > MAX_BASE)
		return 0;

	if (f == 0.0f)
	{
		if (bufsiz >= 4)
		{
			fast_memcpy(buf, "0.0f", 4);
			return 1;
		}
		else
		{
			buf[0] = 0;
			return 0;
		}
	}

	uint32_t bufindex = 0;
	if(f < 0)
	{
		f = -f;
		buf[0] = '-';
		bufindex++;
	}

	int i;
	long fpartmul = 1;
	for(i = 0; i < digits; i++)
		fpartmul *= base;

	long wholepart = (long)f;
	long fpart = (long)((f - wholepart) * fpartmul);

	bufindex += fast_snfmtui(&buf[bufindex], bufsiz - bufindex, wholepart, 10);
	if(bufindex != bufsiz)
	{
		buf[bufindex++] = '.';
	}
	uint32_t tbufindex = bufindex;
	bufindex += fast_snfmtui(&buf[bufindex], bufsiz - bufindex, fpart, 10);
    fast_memmove(&buf[tbufindex + digits - (bufindex - tbufindex)], &buf[tbufindex], (bufindex - tbufindex + 1));
    fast_memset(&buf[tbufindex], '0', digits - (bufindex - tbufindex));
    bufindex = tbufindex + digits;
	return bufindex;
}

unsigned long fast_nextmulof(unsigned long val, unsigned long q)
{
	// base cases
	if(q == 0)
		return 0;
	if(q == 1)
		return val;

	unsigned long modulo = val % q;

	if(modulo == 0)
		return val;

	return (val - modulo) + q;
}

int fast_snfmtui(char* buf, size_t bufsiz, unsigned long i, unsigned long base)
{
    if(bufsiz == 0 || base > MAX_BASE)
        return 0;

    if(i == 0)
    {
        if(bufsiz >= 2)
        {
            buf[0] = __arb_base_digits[0];
            buf[1] = 0;
            return 1;
        }
        else
        {
            buf[0] = 0;
            return 0;
        }
    }

    size_t bufpos = 0, swappos = 0, ret = 0;

    while(bufpos < bufsiz && i != 0)
    {
        buf[bufpos++] = __arb_base_digits[(i % base)];
        i /= base;
    }

    if(bufpos == bufsiz)
    {
        bufpos--;
    }

    buf[bufpos] = '\0';
    ret = bufpos;
    bufpos--;

    while(swappos < bufpos)
    {
        char temp = buf[swappos];
        buf[swappos++] = buf[bufpos];
        buf[bufpos--] = temp;
    }

    return ret;
}

int fast_snfmti(char* buf, size_t bufsiz, long i, unsigned long base)
{
    if(i < 0)
    {
        buf[0] = '-';
        return fast_snfmtui(buf+1, bufsiz-1, -i, base)+1;
    }
    return fast_snfmtui(buf, bufsiz, i, base);
}

__attribute__((format(printf,3,4)))
void fast_snprintf(char* buf, size_t bufsiz, const char* fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);

	size_t bufpos = 0;
	size_t fmtpos = 0;

	size_t fmtlen = fast_strlen(fmt);

	size_t precision = 0;

	bool padwzeros = false;

	bool sym_unsigned = false;

	while(bufpos < bufsiz && fmtpos < fmtlen)
	{
		// Look for escape character
		if(fmt[fmtpos] == '%')
		{
			fmtpos++;
			if (fmtpos == fmtlen)
			{
				buf[bufpos] = '\0';
				return;
			}

			size_t stofprec = fmtpos;

			while('0' <= fmt[fmtpos] && fmt[fmtpos] <= '9')
            {
                if(fmt[fmtpos] == '0' && !padwzeros)
                {
                    padwzeros = true;
                    fmtpos++;
                    stofprec++;
                    continue;
                }

                fmtpos++;
            }

            bool succ;

            precision = fast_sntoul(&fmt[stofprec], fmtpos - stofprec, 10, &succ);
            if(!succ)
            {
                precision = 0;
            }

			_match_fchar:
			switch(fmt[fmtpos])
			{
			case '%':
				// Save '%' to string
				buf[bufpos++] = fmt[fmtpos++];
				continue;
			case 'u':
				// This symbol is unsigned
				sym_unsigned = true;
				fmtpos++;
				goto _match_fchar;
			case 'f':
			{
				float argval;
//				uint32_t tempargval;
//				tempargval = va_arg(ap, uint32_t);
//				argval = *((float*)(&tempargval));
				argval = va_arg(ap, double);
				if(!precision)
                    precision = FLOAT_DEFAULT_DIGITS;
                if(precision > FLOAT_MAX_DIGITS)
                    precision = FLOAT_MAX_DIGITS;
				bufpos += fast_snfmtf(&buf[bufpos], bufsiz-bufpos, argval, precision, 10);
				precision = 0;
				sym_unsigned = false;
				fmtpos++;
				continue;
			}
			case 's':
			{
				char* argval = va_arg(ap, char*);
				fast_strncpy(&buf[bufpos], argval, fast_min(fast_strlen(argval), bufsiz - bufpos));
			}
			case 'd':
			case 'x':
			{
				unsigned long base;
				switch(fmt[fmtpos])
				{
				case 'd':
					base = 10;
					break;
				case 'x':
				    sym_unsigned = true;
					base = 16;
					break;
				}
				// Print an int from the args, formatted as decimal or hex
				if(sym_unsigned)
				{
					unsigned long argval;
					argval = va_arg(ap, unsigned long);
					if(!precision)
                        bufpos += fast_snfmtui(&buf[bufpos], bufsiz-bufpos, argval, base);
                    else
                    {
                        size_t intstrsiz = fast_snfmtui(&buf[bufpos], bufsiz-bufpos, argval, base);
                        if(intstrsiz < precision)
                        {
                            if((bufsiz-bufpos) >= precision)
                            {
                                fast_memmove(&buf[bufpos + (precision - intstrsiz)], &buf[bufpos], intstrsiz);
                                fast_memset(&buf[bufpos], (uint8_t)((padwzeros)?('0'):(' ')), precision-intstrsiz);
                                bufpos += precision;
                            }
                            else
                                break;
                        }
                        else
                            bufpos += intstrsiz;
                    }
					sym_unsigned = false;
				}
				else
				{
					long argval;
					argval = va_arg(ap, long);
					if(!precision)
                        bufpos += fast_snfmti(&buf[bufpos], bufsiz-bufpos, argval, base);
                    else
                    {
                        size_t intstrsiz = fast_snfmti(&buf[bufpos], bufsiz-bufpos, argval, base);
                        if(intstrsiz < precision + 1)
                        {
                            if((bufsiz-bufpos) >= precision)
                            {
                                fast_memmove(&buf[bufpos + (precision - intstrsiz) + 2], &buf[bufpos + 1], intstrsiz);
                                fast_memset(&buf[bufpos + 1], (uint8_t)((padwzeros)?('0'):(' ')), precision-intstrsiz + 1);
                                bufpos += precision + 1;
                            }
                            else
                                break;
                        }
                        else
                            bufpos += intstrsiz;
                    }
				}
				fmtpos++;
				continue;
			}
			}
		}
		buf[bufpos++] = fmt[fmtpos++];
	}

	if(bufpos == bufsiz)
		bufpos--;
	buf[bufpos] = '\0';

	va_end(ap);
}

