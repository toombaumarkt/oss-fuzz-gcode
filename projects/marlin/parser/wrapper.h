
#ifndef WRAPPER_H
#define WRAPPER_H

#include <stdint.h>
#include <string.h>
#include <iostream>

#include "macros.h"
#include "Configuration.h"
#include "Configuration_adv.h"

#include "language.h"

#include "types.h"
#include "Conditionals_LCD.h"


// fuzzing parameters
#define MAX_DATA_SIZE 1000
#define MIN_DATA_SIZE 1

#define DEBUG_OUTPUT 0



#define USE_GCODE_SUBCODES 1
#define GCODE_MOTION_MODES 1
#define FASTER_GCODE_PARSER 1
#define DEBUG_GCODE_PARSER 0

// copied from millis_t.h
typedef uint32_t millis_t;
#define SEC_TO_MS(N) millis_t((N)*1000UL)
#define MIN_TO_MS(N) SEC_TO_MS((N)*60UL)
#define MS_TO_SEC(N) millis_t((N)/1000UL)


// copied from Arduino.h
#define constrain(value, arg_min, arg_max) ((value) < (arg_min) ? (arg_min) :((value) > (arg_max) ? (arg_max) : (value)))
#define PGM_P const char *



// copied from serial_base.h
struct serial_index_t {
  // A signed index, where -1 is a special case meaning no action (neither output or input)
  int8_t  index;

  // Check if the index is within the range [a ... b]
  constexpr inline bool within(const int8_t a, const int8_t b) const { return WITHIN(index, a, b); }
  constexpr inline bool valid() const { return WITHIN(index, 0, 7); } // At most, 8 bits

  // Construction is either from an index
  constexpr serial_index_t(const int8_t index) : index(index) {}

  // Default to "no index"
  constexpr serial_index_t() : index(-1) {}
};

// copied from HAL.h
// String helper
#ifndef PGMSTR
  #define PGMSTR(NAM,STR) const char NAM[] = STR
#endif

#endif