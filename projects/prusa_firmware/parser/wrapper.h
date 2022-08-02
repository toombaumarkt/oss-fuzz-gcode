
#ifndef WRAPPER_H
#define WRAPPER_H

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <iostream>
#include <chrono>
#include <time.h>

#include "macros.h"
#include "Configuration.h"
#include "Configuration_adv.h"
#include "1_75mm_MK3S-EINSy10a-E3Dv6full.h"


#include "language.h"
#include "new_marlin_main.h"
#include "cmdqueue.h"



// fuzzing parameters
#define MAX_DATA_SIZE 1000
#define MIN_DATA_SIZE 1

#define DEBUG_OUTPUT 1



#define USE_GCODE_SUBCODES 1
#define GCODE_MOTION_MODES 1
#define FASTER_GCODE_PARSER 1
#define DEBUG_GCODE_PARSER 0


// copied from eeprom.h
#define EEPROM_EMPTY_VALUE 0xFF

// copied from millis_t.h
typedef uint32_t millis_t;
#define SEC_TO_MS(N) millis_t((N)*1000UL)
#define MIN_TO_MS(N) SEC_TO_MS((N)*60UL)
#define MS_TO_SEC(N) millis_t((N)/1000UL)


// copied from Arduino.h
#define constrain(value, arg_min, arg_max) ((value) < (arg_min) ? (arg_min) :((value) > (arg_max) ? (arg_max) : (value)))
#define PGM_P const char *


// copied from cardreader.h
extern bool Stopped;

// copied from ultralcd.h
extern uint8_t farm_mode;

// copied from HAL.h
// String helper
#ifndef PGMSTR
  #define PGMSTR(NAM,STR) const char NAM[] = STR
#endif

// copied from Marlin.h
enum AxisEnum {X_AXIS=0, Y_AXIS=1, Z_AXIS=2, E_AXIS=3, X_HEAD=4, Y_HEAD=5};
#define X_AXIS_MASK  1
#define Y_AXIS_MASK  2
#define Z_AXIS_MASK  4
#define E_AXIS_MASK  8
#define X_HEAD_MASK 16
#define Y_HEAD_MASK 32

enum class HeatingStatus : uint8_t
{
    NO_HEATING = 0,
    EXTRUDER_HEATING = 1,
    EXTRUDER_HEATING_COMPLETE = 2,
    BED_HEATING = 3,
    BED_HEATING_COMPLETE = 4,
};

// copied from cardreader.h
extern bool Stopped;

// copied from ConfiguratiuonStore.h
typedef struct
{
    char version[4];
    float axis_steps_per_unit[4];
    float max_feedrate_normal[4];
    unsigned long max_acceleration_units_per_sq_second_normal[4];
    float acceleration; //!< Normal acceleration mm/s^2  THIS IS THE DEFAULT ACCELERATION for all moves. M204 SXXXX
    float retract_acceleration; //!< mm/s^2 filament pull-pack and push-forward while standing still in the other axis M204 TXXXX
    float minimumfeedrate;
    float mintravelfeedrate;
    unsigned long minsegmenttime;
    float max_jerk[4]; //!< Jerk is a maximum immediate velocity change.
    float add_homing[3];
    float zprobe_zoffset;
    float Kp;
    float Ki;
    float Kd;
    float bedKp;
    float bedKi;
    float bedKd;
    int lcd_contrast; //!< unused
    bool autoretract_enabled;
    float retract_length;
    float retract_feedrate;
    float retract_zlift;
    float retract_recover_length;
    float retract_recover_feedrate;
    bool volumetric_enabled;
    float filament_size[1]; //!< cross-sectional area of filament (in millimeters), typically around 1.75 or 2.85, 0 disables the volumetric calculations for the extruder.
    float max_feedrate_silent[4]; //!< max speeds for silent mode
    unsigned long max_acceleration_units_per_sq_second_silent[4];
    unsigned char axis_ustep_resolution[4];
    float travel_acceleration; //!< travel acceleration mm/s^2
    // Arc Interpolation Settings, configurable via M214
    float mm_per_arc_segment;
    float min_mm_per_arc_segment;
    unsigned char n_arc_correction; // If equal to zero, this is disabled
    unsigned short min_arc_segments; // If equal to zero, this is disabled
    unsigned short arc_segments_per_sec; // If equal to zero, this is disabled
} M500_conf;

// copied from planner.h
inline void set_current_to_destination() { memcpy(current_position, destination, sizeof(current_position)); }
inline void set_destination_to_current() { memcpy(destination, current_position, sizeof(destination)); }

// copied from Marlin.h
#define PRINT_PERCENT_DONE_INIT 0xff
#define PRINT_TIME_REMAINING_INIT 0xffff
#ifndef analogInputToDigitalPin
#define analogInputToDigitalPin(p) ((p) + A0)
#endif

// copied from pins.h and pins_Einsy_1_0.h
#define X_TMC2130_CS           41
#define X_TMC2130_DIAG         64 // !!! changed from 40 (EINY03)
#define X_STEP_PIN             37
#define X_DIR_PIN              49
#define X_MIN_PIN            12
//#define X_MAX_PIN            30
//#define X_MIN_PIN              X_TMC2130_DIAG
#define X_MAX_PIN              X_TMC2130_DIAG
#define X_ENABLE_PIN           29
#define X_MS1_PIN           -1
#define X_MS2_PIN           -1

#define Y_TMC2130_CS        39
#define Y_TMC2130_DIAG      69
#define Y_STEP_PIN          36
#define Y_DIR_PIN           48
#define Y_MIN_PIN           11
//#define Y_MAX_PIN           24
//#define Y_MIN_PIN           Y_TMC2130_DIAG
#define Y_MAX_PIN           Y_TMC2130_DIAG
#define Y_ENABLE_PIN        28
#define Y_MS1_PIN           -1
#define Y_MS2_PIN           -1

#define Z_TMC2130_CS        67
#define Z_TMC2130_DIAG      68
#define Z_STEP_PIN          35
#define Z_DIR_PIN           47
#define Z_MIN_PIN           10
#define Z_MAX_PIN           23
//#define Z_MAX_PIN           Z_TMC2130_DIAG
#define Z_ENABLE_PIN        27
#define Z_MS1_PIN           -1
#define Z_MS2_PIN           -1

#define HEATER_BED_PIN       4 //PG5
#define TEMP_BED_PIN         2 //A2

#define HEATER_0_PIN         3 //PE5
#define TEMP_0_PIN           0 //A0

#define HEATER_1_PIN        -1
#define TEMP_1_PIN           1 //A1

#define HEATER_2_PIN        -1
#define TEMP_2_PIN          -1

#define TEMP_AMBIENT_PIN     6 //A6

#define TEMP_PINDA_PIN       3 //A3

#define VOLT_PWR_PIN         4 //A4
#define VOLT_BED_PIN         9 //A9
#define VOLT_IR_PIN          8 //A8


#define E0_TMC2130_CS       66
#define E0_TMC2130_DIAG     65
#define E0_STEP_PIN         34
#define E0_DIR_PIN          43
#define E0_ENABLE_PIN       26
#define E0_MS1_PIN          -1
#define E0_MS2_PIN          -1

#define SDPOWER             -1
#define SDSS                77
#define LED_PIN             13
#define FAN_PIN              6
#define FAN_1_PIN           -1
#define PS_ON_PIN           -1
#define KILL_PIN            -1  // 80 with Smart Controller LCD
#define SUICIDE_PIN         -1  // PIN that has to be turned on right after start, to keep power flowing.

//List of pins which to ignore when asked to change by gcode, 0 and 1 are RX and TX, do not mess with those!
#define _E0_PINS E0_STEP_PIN, E0_DIR_PIN, E0_ENABLE_PIN, HEATER_0_PIN,
#if EXTRUDERS > 1
  #define _E1_PINS E1_STEP_PIN, E1_DIR_PIN, E1_ENABLE_PIN, HEATER_1_PIN,
#else
  #define _E1_PINS
#endif
#if EXTRUDERS > 2
  #define _E2_PINS E2_STEP_PIN, E2_DIR_PIN, E2_ENABLE_PIN, HEATER_2_PIN,
#else
  #define _E2_PINS
#endif

#define SENSITIVE_PINS {0, 1, X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, X_MIN_PIN, X_MAX_PIN, Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, Y_MIN_PIN, Y_MAX_PIN, Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, Z_MIN_PIN, Z_MAX_PIN, PS_ON_PIN, \
                        HEATER_BED_PIN, FAN_PIN,                  \
                        _E0_PINS _E1_PINS _E2_PINS   /*          \
                        analogInputToDigitalPin(TEMP_0_PIN), analogInputToDigitalPin(TEMP_1_PIN), analogInputToDigitalPin(TEMP_2_PIN), analogInputToDigitalPin(TEMP_BED_PIN)*/ }

// copied from util.h
enum class ClPrintChecking:uint_least8_t
{
    _Nozzle=1,
    _Model=2,
    _Smodel=3,
    _Version=4,
    _Gcode=5
};


// copied from printers.h
// *** MK3S
#define PRINTER_MK3S     		302
#define PRINTER_MK3S_NAME          "MK3S"
#define PRINTER_MK3S_MMU2          20302
#define PRINTER_MK3S_MMU2_NAME     "MK3SMMU2S"


#endif