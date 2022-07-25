
#ifndef WRAPPER_H
#define WRAPPER_H


#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "nuts_bolts.h"
#include "config.h"
#include "settings.h"

#include "report.h"
#include "gcode.h"
#include "protocol.h"


// fuzzing parameters
#define MAX_DATA_SIZE 1000
#define MIN_DATA_SIZE 1
#define DEBUG_OUTPUT 0


#define false 0
#define true 1

// Axis array index values. Must start with 0 and be continuous.
#define N_AXIS 3 // Number of axes
#define X_AXIS 0 // Axis indexing value. 
#define Y_AXIS 1
#define Z_AXIS 2
// #define A_AXIS 3


// copied from system.h
// Define global system variables
typedef struct {
  uint8_t abort;                 // System abort flag. Forces exit back to main loop for reset.
  uint8_t state;                 // Tracks the current state of Grbl.
  uint8_t suspend;               // System suspend bitflag variable that manages holds, cancels, and safety door.
  uint8_t soft_limit;            // Tracks soft limit errors for the state machine. (boolean)
  
  int32_t position[N_AXIS];      // Real-time machine (aka home) position vector in steps. 
                                 // NOTE: This may need to be a volatile variable, if problems arise.                             

  int32_t probe_position[N_AXIS]; // Last probe position in machine coordinates and steps.
  uint8_t probe_succeeded;        // Tracks if last probing cycle was successful.
  uint8_t homing_axis_lock;       // Locks axes when limits engage. Used as an axis motion mask in the stepper ISR.
} system_t;
extern system_t sys;

//copied from defaults_generic.h
  // Grbl generic default settings. Should work across different machines.
  #define DEFAULT_X_STEPS_PER_MM 250.0
  #define DEFAULT_Y_STEPS_PER_MM 250.0
  #define DEFAULT_Z_STEPS_PER_MM 250.0
  #define DEFAULT_X_MAX_RATE 500.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 500.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 500.0 // mm/min
  #define DEFAULT_X_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 200.0 // mm
  #define DEFAULT_Y_MAX_TRAVEL 200.0 // mm
  #define DEFAULT_Z_MAX_TRAVEL 200.0 // mm
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK 0
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 25 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK ((BITFLAG_RT_STATUS_MACHINE_POSITION)|(BITFLAG_RT_STATUS_WORK_POSITION))
  #define DEFAULT_JUNCTION_DEVIATION 0.01 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_HOMING_ENABLE 0  // false
  #define DEFAULT_HOMING_DIR_MASK 0 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE 25.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 500.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 1.0 // mm

//copied from protocol.h
#ifndef LINE_BUFFER_SIZE
  #define LINE_BUFFER_SIZE 80
#endif
// Define different comment types for pre-parsing.
#define COMMENT_NONE 0
#define COMMENT_TYPE_PARENTHESES 1
#define COMMENT_TYPE_SEMICOLON 2


//copied from cpu_map....h
// Define step pulse output pins. NOTE: All step bit pins must be on the same port.
#define STEP_DDR        DDRD
#define STEP_PORT       PORTD
#define X_STEP_BIT      2  // Uno Digital Pin 2
#define Y_STEP_BIT      3  // Uno Digital Pin 3
#define Z_STEP_BIT      4  // Uno Digital Pin 4
#define STEP_MASK       ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)) // All step bits

// Define step direction output pins. NOTE: All direction pins must be on the same port.
#define DIRECTION_DDR     DDRD
#define DIRECTION_PORT    PORTD
#define X_DIRECTION_BIT   5  // Uno Digital Pin 5
#define Y_DIRECTION_BIT   6  // Uno Digital Pin 6
#define Z_DIRECTION_BIT   7  // Uno Digital Pin 7
#define X_LIMIT_BIT      1  // Uno Digital Pin 9
#define Y_LIMIT_BIT      2  // Uno Digital Pin 10
#ifdef VARIABLE_SPINDLE // Z Limit pin and spindle enabled swapped to access hardware PWM on Pin 11.  
  #define Z_LIMIT_BIT	   4 // Uno Digital Pin 12
#else
  #define Z_LIMIT_BIT    3  // Uno Digital Pin 11
#endif

// copied from serial.h
#define SERIAL_NO_DATA 0xff

// copied from system.h
// Define system state bit map. The state variable primarily tracks the individual functions
// of Grbl to manage each without overlapping. It is also used as a messaging flag for
// critical events.
#define STATE_IDLE          0      // Must be zero. No flags.
#define STATE_ALARM         bit(0) // In alarm state. Locks out all g-code processes. Allows settings access.
#define STATE_CHECK_MODE    bit(1) // G-code check mode. Locks out planner and motion only.
#define STATE_HOMING        bit(2) // Performing homing cycle
#define STATE_CYCLE         bit(3) // Cycle is running or motions are being executed.
#define STATE_HOLD          bit(4) // Active feed hold
#define STATE_SAFETY_DOOR   bit(5) // Safety door is ajar. Feed holds and de-energizes system.
#define STATE_MOTION_CANCEL bit(6) // Motion cancel by feed hold and return to idle. 


#endif
