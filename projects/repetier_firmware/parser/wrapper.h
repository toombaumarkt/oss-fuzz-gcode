
#ifndef WRAPPER_H
#define WRAPPER_H

// fuzzing parameters
#define MAX_DATA_SIZE 1000
#define MIN_DATA_SIZE 1

#define DEBUG_OUTPUT 0
#define FUZZ_TARGET_1_EASY_SANITY_CHECK 0
#define FUZZ_TARGET_1_HARD_SANITY_CHECK 0
#define ENABLE_CHECKSUM_CALC 1

// define target function to fuzz
// 0 -> parseAscii
// 1 -> parseBinary
//#define FUZZ_TARGET 1




// includes
#include <math.h>
#include <stdint.h>


#define uint uint16_t
#define uint8 uint8_t
#define int8 int8_t
#define uint32 uint32_t
#define int32 int32_t


#ifndef KEEP_ALIVE_INTERVAL
#define KEEP_ALIVE_INTERVAL 2000
#endif

typedef unsigned int speed_t;
typedef unsigned long ticks_t;
typedef unsigned long millis_t;
typedef unsigned int flag8_t;
typedef int fast8_t;
typedef unsigned int ufast8_t;

#define PGM_P const char*
#define FSTRINGPARAM(var) PGM_P var
#define FSTRINGVAR(var) static const char var[];
#define FSTRINGVALUE(var, value) const char var[] = value;


#define ECHO_ON_EXECUTE 1
#define GCODE_BUFFER_SIZE 1
#define NEW_COMMUNICATION 1

#undef min
#undef max

class RMath {
public:
    static inline float min(float a, float b) {
        if (a < b)
            return a;
        return b;
    }
    static inline float max(float a, float b) {
        if (a < b)
            return b;
        return a;
    }
    static inline int32_t min(int32_t a, int32_t b, int32_t c) {
        if (a < b)
            return a < c ? a : c;
        return b < c ? b : c;
    }
    static inline float min(float a, float b, float c) {
        if (a < b)
            return a < c ? a : c;
        return b < c ? b : c;
    }
    static inline int32_t max(int32_t a, int32_t b) {
        if (a < b)
            return b;
        return a;
    }
    static inline int min(int a, int b) {
        if (a < b)
            return a;
        return b;
    }
    static inline uint16_t min(uint16_t a, uint16_t b) {
        if (a < b)
            return a;
        return b;
    }
    static inline int16_t max(int16_t a, int16_t b) {
        if (a < b)
            return b;
        return a;
    }
    static inline uint16_t max(uint16_t a, uint16_t b) {
        if (a < b)
            return b;
        return a;
    }
    static inline unsigned long absLong(long a) { return a >= 0 ? a : -a; }
    static inline int32_t sqr(int32_t a) { return a * a; }
    static inline uint32_t sqr(uint32_t a) { return a * a; }
#ifdef SUPPORT_64_BIT_MATH
    static inline int64_t sqr(int64_t a) {
        return a * a;
    }
    static inline uint64_t sqr(uint64_t a) { return a * a; }
#endif

    static inline float sqr(float a) {
        return a * a;
    }
};

#endif
