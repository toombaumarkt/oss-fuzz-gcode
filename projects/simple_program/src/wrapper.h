
#ifndef WRAPPER_H
#define WRAPPER_H

// fuzzing parameters
#define MAX_DATA_SIZE 1000
#define MIN_DATA_SIZE 1

#define DEBUG_OUTPUT 0
#define FUZZ_TARGET_1_EASY_SANITY_CHECK 0
#define FUZZ_TARGET_1_HARD_SANITY_CHECK 0
#define ENABLE_CHECKSUM_CALC 0

// define target function to fuzz
// 0 -> parseAscii
// 1 -> parseBinary
//#define FUZZ_TARGET 1




// includes
#include <math.h>
#include <stdint.h>

#endif
