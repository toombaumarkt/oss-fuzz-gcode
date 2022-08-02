#ifndef MARLIN_H
#define MARLIN_H

#include <bsd/string.h>
#include "Configuration.h"

extern bool fans_check_enabled;
extern float homing_feedrate[];
extern uint8_t axis_relative_modes;
extern float feedrate;
extern int feedmultiply;
extern int extrudemultiply; // Sets extrude multiply factor (in percent) for all extruders
extern int extruder_multiply[EXTRUDERS]; // sets extrude multiply factor (in percent) for each extruder individually
extern float extruder_multiplier[EXTRUDERS]; // reciprocal of cross-sectional area of filament (in square millimeters), stored this way to reduce computational burden in planner
extern float current_position[NUM_AXIS] ;
extern float destination[NUM_AXIS] ;
extern float min_pos[3];
extern float max_pos[3];
extern bool axis_known_position[3];
extern int fanSpeed;
extern uint8_t newFanSpeed;
extern int8_t lcd_change_fil_state;
extern float default_retraction;


#define NUM_AXIS 4 // The axis order in all axis related arrays is X, Y, Z, E

//put an ASCII command at the end of the current buffer, read from flash
#define enquecommand_P(cmd) enquecommand(cmd, true)

//put an ASCII command at the begin of the current buffer, read from flash
#define enquecommand_front_P(cmd) enquecommand_front(cmd, true)


// define PSTR to do nothing, so I don't have to remove every single one :)
#define PSTR(str) str

long millis();

void process_commands();

void get_coordinates();

#endif