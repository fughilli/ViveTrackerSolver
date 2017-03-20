#ifndef _FAST_UTILS_H_
#define _FAST_UTILS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#ifndef NULL
#define NULL ((void*)0)
#endif

#define ATOMIC

//#ifndef size_t
//typedef uint32_t size_t;
//#endif

#define fast_min(_a_,_b_) (((_a_) < (_b_)) ? (_a_) : (_b_))
#define fast_max(_a_,_b_) (((_a_) > (_b_)) ? (_a_) : (_b_))

#define FAST_USE_ALLOC (false)

#define __ALLOC_MEM_SIZE (4096)

#define FLOAT_DEFAULT_DIGITS (4)
#define FLOAT_MAX_DIGITS (8)

void fast_memcpy(void* dst, const void* src, size_t sz);
void fast_memmove(void* dst, const void* src, size_t sz);
void fast_memset(void* dst, uint8_t val, size_t sz);
int fast_memcmp(const void* ptr1, const void* ptr2, size_t sz);

size_t fast_strlen(const char* str);
size_t fast_strnlen(const char* str, size_t maxlen);
size_t fast_strcpy(char* stra, const char* strb);
size_t fast_strncpy(char* stra, const char* strb, size_t maxlen);
int fast_strcmp(const char* stra, const char* strb);


__attribute__((format(printf,3,4)))
void fast_snprintf(char* buf, size_t bufsiz, const char* fmt, ...);

long fast_sntol(const char* str, size_t sz, unsigned int base, bool* succ);
unsigned long fast_sntoul(const char* str, size_t sz, unsigned int base, bool* succ);
float fast_sntof(const char* str, size_t sz, unsigned int base, bool* succ);
double fast_sntod(const char* str, size_t sz, unsigned int base, bool* succ);

bool fast_sntob(const char* str, size_t sz, bool* succ);

int fast_snfmtui(char* buf, size_t bufsiz, unsigned long i, unsigned long base);
int fast_snfmti(char* buf, size_t bufsiz, long i, unsigned long base);

int fast_snfmtf(char* buf, size_t bufsiz, float f, size_t digits, unsigned long base);

unsigned long fast_nextmulof(unsigned long val, unsigned long q);

void* fast_alloc(size_t sz);
void fast_free(void* buf);
void init_heap();

void fast_swap8(uint8_t* a, uint8_t* b);
void fast_swap16(uint16_t* a, uint16_t* b);
void fast_swap32(uint32_t* a, uint32_t* b);

#ifdef __cplusplus
}
#endif

#endif // _FAST_UTILS_H_


