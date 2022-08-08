
#include "wrapper.h"
#include "new_marlin_main.h"
#include "Dcodes.h"

M500_conf cs;

uint8_t farm_mode = 0;

// copied from cardreader.cpp
bool Stopped=false;

// copied from Configuration.cpp
PGM_P sPrinterName;

//copied from planner.h
// Use M203 to override by software
float* max_feedrate = cs.max_feedrate_normal;
// Use M201 to override by software
unsigned long* max_acceleration_units_per_sq_second = cs.max_acceleration_units_per_sq_second_normal;
unsigned long axis_steps_per_sqr_second[NUM_AXIS];

uint32_t IP_address = 0;
bool isPrintPaused = false;

// storing estimated time to end of print counted by slicer
uint8_t print_percent_done_normal = PRINT_PERCENT_DONE_INIT;
uint8_t print_percent_done_silent = PRINT_PERCENT_DONE_INIT;
uint16_t print_time_remaining_normal = PRINT_TIME_REMAINING_INIT; //estimated remaining print time in minutes
uint16_t print_time_remaining_silent = PRINT_TIME_REMAINING_INIT; //estimated remaining print time in minutes
uint16_t print_time_to_change_normal = PRINT_TIME_REMAINING_INIT; //estimated remaining time to next change in minutes
uint16_t print_time_to_change_silent = PRINT_TIME_REMAINING_INIT; //estimated remaining time to next change in minutes

//filament types 
#define FILAMENT_DEFAULT 0
#define FILAMENT_FLEX 1
#define FILAMENT_PVA 2
#define FILAMENT_UNDEFINED 255


//Inactivity shutdown variables
//static LongTimer previous_millis_cmd;
unsigned long max_inactive_time = 0;
static unsigned long stepper_inactive_time = DEFAULT_STEPPER_DEACTIVE_TIME*1000l;
static unsigned long safetytimer_inactive_time = DEFAULT_SAFETYTIMER_TIME_MINS*60*1000ul;

static int saved_feedmultiply_mm = 100;


const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};
float destination[NUM_AXIS] = {  0.0, 0.0, 0.0, 0.0};

// For tracing an arc
static float offset[3] = {0.0, 0.0, 0.0};

// Current feedrate
float feedrate = 1500.0;

// Feedrate for the next move
static float next_feedrate;

// Original feedrate saved during homing moves
static float saved_feedrate;

uint8_t status_number = 0;

unsigned long total_filament_used;
HeatingStatus heating_status;
uint8_t heating_status_counter;
bool loading_flag = false;

unsigned long PingTime = millis();
bool no_response = false;
uint16_t gcode_in_progress = 0;
uint16_t mcode_in_progress = 0;

//Although this flag and many others like this could be represented with a struct/bitfield for each axis (more readable and efficient code), the implementation
//would not be standard across all platforms. That being said, the code will continue to use bitmasks for independent axis.
//Moreover, according to C/C++ standard, the ordering of bits is platform/compiler dependent and the compiler is allowed to align the bits arbitrarily,
//thus bit operations like shifting and masking may stop working and will be very hard to fix.
uint8_t axis_relative_modes = 0;

int feedmultiply=100; //100->1 200->2
int extrudemultiply=100; //100->1 200->2
int extruder_multiply[EXTRUDERS] = {100
  #if EXTRUDERS > 1
    , 100
    #if EXTRUDERS > 2
      , 100
    #endif
  #endif
};


#ifdef FWRETRACT
  bool retracted[EXTRUDERS]={false
    #if EXTRUDERS > 1
    , false
     #if EXTRUDERS > 2
      , false
     #endif
  #endif
  };
  bool retracted_swap[EXTRUDERS]={false
    #if EXTRUDERS > 1
    , false
     #if EXTRUDERS > 2
      , false
     #endif
  #endif
  };

  float retract_length_swap = RETRACT_LENGTH_SWAP;
  float retract_recover_length_swap = RETRACT_RECOVER_LENGTH_SWAP;
#endif

  #ifdef PS_DEFAULT_OFF
    bool powersupply = false;
  #else
	  bool powersupply = true;
  #endif

bool cancel_heatup = false;

unsigned long starttime=0;
unsigned long stoptime=0;

bool fan_state[2];
int fan_edge_counter[2];
int fan_speed[2];

uint8_t active_extruder = 0;
int fanSpeed=0;
uint8_t newFanSpeed = 0;

float extruder_multiplier[EXTRUDERS] = {1.0
  #if EXTRUDERS > 1
    , 1.0
    #if EXTRUDERS > 2
      , 1.0
    #endif
  #endif
};
const int8_t sensitive_pins[] = SENSITIVE_PINS; // Sensitive pin list for M42

float current_position[NUM_AXIS] = { 0.0, 0.0, 0.0, 0.0 };
//shortcuts for more readable code
#define _x current_position[X_AXIS]
#define _y current_position[Y_AXIS]
#define _z current_position[Z_AXIS]
#define _e current_position[E_AXIS]

float min_pos[3] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS };
float max_pos[3] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };
bool axis_known_position[3] = {false, false, false};


// Define some coordinates outside the clamp limits (making them invalid past the parsing stage) so
// that they can be used later for various logical checks
#define X_COORD_INVALID (X_MIN_POS-1)
#define Y_COORD_INVALID (Y_MIN_POS-1)

#define SAVED_TARGET_UNSET X_COORD_INVALID
float saved_target[NUM_AXIS] = {SAVED_TARGET_UNSET, 0, 0, 0};

bool homing_flag = false;


// copied from utils.cpp
#define PRINTER_NAME_LENGTH ((sizeof(PRINTER_MMU_NAME)>sizeof(PRINTER_NAME))?(sizeof(PRINTER_MMU_NAME)-1):(sizeof(PRINTER_NAME)-1))
#define GCODE_DELIMITER '"'
#define ELLIPSIS "..."
enum class ClCheckVersion:uint_least8_t
{
    _None,
    _Warn,
    _Strict,
    _Undef=EEPROM_EMPTY_VALUE
};
ClCheckVersion oCheckVersion=ClCheckVersion::_None;



/**
 * @brief FUZZING -- added reset function to assign initial values to static members
 * 
 */
void reset(){
  min_pos[0] = X_MIN_POS;
  min_pos[1] = Y_MIN_POS;
  min_pos[2] = Z_MIN_POS;

  max_pos[0] = X_MAX_POS;
  max_pos[1] = Y_MAX_POS;
  max_pos[2] = Z_MAX_POS;

  axis_known_position[0] = false;
  axis_known_position[1] = false;
  axis_known_position[2] = false;

  memset(current_position, 0, sizeof(current_position));
  
  extruder_multiplier[0] = {1.0};
  #if EXTRUDERS > 1
    extruder_multiplier[1] = 1.0;
    #if EXTRUDERS > 2
      extruder_multiplier[2] = 1.0;
    #endif
  #endif

  active_extruder = 0;
  fanSpeed= 0;
  newFanSpeed = 0;

  memset(fan_state, 0, sizeof(fan_state));
  fan_edge_counter[0] = 0;
  fan_edge_counter[1] = 0;
  fan_speed[0] = 0;
  fan_speed[1] = 0;

  starttime=0;
  stoptime=0;

  cancel_heatup = false;

  #ifdef PS_DEFAULT_OFF
    powersupply = false;
  #else
	  powersupply = true;
  #endif

  #ifdef FWRETRACT
  retracted[0]= false;
    #if EXTRUDERS > 1
    retracted[1]= false;
     #if EXTRUDERS > 2
      retracted[2]= false;
     #endif
  #endif
  
  retracted_swap[0]= false;
    #if EXTRUDERS > 1
    retracted_swap[1]= false;
     #if EXTRUDERS > 2
      retracted_swap[2]= false;
     #endif
  #endif
  

  retract_length_swap = RETRACT_LENGTH_SWAP;
  retract_recover_length_swap = RETRACT_RECOVER_LENGTH_SWAP;
#endif

  axis_relative_modes = 0;

  feedmultiply=100; //100->1 200->2
  extrudemultiply=100; //100->1 200->2
  extruder_multiply[0] = 100;
  #if EXTRUDERS > 1
    extruder_multiply[1] = 100;
    #if EXTRUDERS > 2
      extruder_multiply[1] = 100;
    #endif
  #endif

  max_inactive_time = 0;
  stepper_inactive_time = DEFAULT_STEPPER_DEACTIVE_TIME*1000l;
  safetytimer_inactive_time = DEFAULT_SAFETYTIMER_TIME_MINS*60*1000ul;

  saved_feedmultiply_mm = 100;

  memset(destination, 0, sizeof(destination));

// For tracing an arc
  memset(offset, 0, sizeof(offset));

// Current feedrate
  feedrate = 1500.0;

// Feedrate for the next move
  next_feedrate = 0;

// Original feedrate saved during homing moves
  saved_feedrate = 0;

  status_number = 0;

  total_filament_used = 0;
  memset(&heating_status, 0, sizeof(heating_status));
  heating_status_counter = 0;
  loading_flag = false;

  PingTime = millis();
  no_response = false;
  gcode_in_progress = 0;
  mcode_in_progress = 0;

  M500_conf cs;

  farm_mode = 0;

// copied from cardreader.cpp
  Stopped=false;

//copied from planner.h
// Use M203 to override by software
  max_feedrate = cs.max_feedrate_normal;
// Use M201 to override by software
  max_acceleration_units_per_sq_second = cs.max_acceleration_units_per_sq_second_normal;
  memset(axis_steps_per_sqr_second, 0, sizeof(axis_steps_per_sqr_second));


  IP_address = 0;
  isPrintPaused = false;

// storing estimated time to end of print counted by slicer
  print_percent_done_normal = PRINT_PERCENT_DONE_INIT;
  print_percent_done_silent = PRINT_PERCENT_DONE_INIT;
  print_time_remaining_normal = PRINT_TIME_REMAINING_INIT; //estimated remaining print time in minutes
  print_time_remaining_silent = PRINT_TIME_REMAINING_INIT; //estimated remaining print time in minutes
  print_time_to_change_normal = PRINT_TIME_REMAINING_INIT; //estimated remaining time to next change in minutes
  print_time_to_change_silent = PRINT_TIME_REMAINING_INIT; //estimated remaining time to next change in minutes

  oCheckVersion=ClCheckVersion::_None;
}


char* code_string(char* pStr,size_t* nLength)
{
char* pStrBegin;
char* pStrEnd;

pStrBegin=strchr(pStr,GCODE_DELIMITER);
if(!pStrBegin)
     return(NULL);
pStrBegin++;
pStrEnd=strchr(pStrBegin,GCODE_DELIMITER);
if(!pStrEnd)
     return(NULL);
*nLength=pStrEnd-pStrBegin;
return(pStrBegin);
}

void printer_smodel_check(char* pStrPos)
{
char* pResult;
size_t nLength,nPrinterNameLength;
bool bCheckOK;
char sPrinterName[PRINTER_NAME_LENGTH+sizeof(ELLIPSIS)-1+1]="";

nPrinterNameLength=strlen(::sPrinterName);
pResult=code_string(pStrPos,&nLength);
if(pResult!=NULL)
     {
     strlcpy(sPrinterName,pResult,std::min(nPrinterNameLength,nLength)+1);
     if(nLength>nPrinterNameLength)
          strcat(sPrinterName,ELLIPSIS);
     bCheckOK=(nLength==nPrinterNameLength);
     if(bCheckOK&&(!strncasecmp(pResult,::sPrinterName,nLength))) // i.e. string compare execute only if lengths are same
          return;
     }
//SERIAL_ECHO_START;
//SERIAL_ECHOLNPGM("Printer model differs from the G-code ...");
//SERIAL_ECHOPGM("actual  : \"");
//serialprintPGM(::sPrinterName);
//SERIAL_ECHOLNPGM("\"");
//SERIAL_ECHOPGM("expected: \"");
////SERIAL_ECHO(sPrinterName);
//SERIAL_ECHOLNPGM("\"");
/* switch(oCheckModel)
     {
     case ClCheckModel::_Warn:
//          lcd_show_fullscreen_message_and_wait_P(_i("Printer model differs from the G-code. Continue?"));
lcd_display_message_fullscreen_P(_T(MSG_GCODE_DIFF_PRINTER_CONTINUE));
lcd_wait_for_click_delay(MSG_PRINT_CHECKING_FAILED_TIMEOUT);
//???custom_message_type=CUSTOM_MSG_TYPE_STATUS; // display / status-line recovery
lcd_update_enable(true);           // display / status-line recovery
          break;
     case ClCheckModel::_Strict:
          lcd_show_fullscreen_message_and_wait_P(_T(MSG_GCODE_DIFF_PRINTER_CANCELLED));
          lcd_print_stop();
          break;
     case ClCheckModel::_None:
     case ClCheckModel::_Undef:
          break;
     } */
}


// copied from utils.cpp
// Definition of a firmware flavor numerical values.
enum FirmwareRevisionFlavorType
{
    FIRMWARE_REVISION_DEV = 0,
    FIRMWARE_REVISION_ALPHA = 1,
    FIRMWARE_REVISION_BETA = 2,
    FIRMWARE_REVISION_RC,
    FIRMWARE_REVISION_RC2,
    FIRMWARE_REVISION_RC3,
    FIRMWARE_REVISION_RC4,
    FIRMWARE_REVISION_RC5,
    FIRMWARE_REVISION_RELEASED = 127
};



// Allocate the version string in the program memory. Otherwise the string lands either on the stack or in the global RAM.
static const char FW_VERSION_STR[] = FW_VERSION;
static const uint16_t FW_VERSION_NR[4] = { FW_MAJOR, FW_MINOR, FW_REVISION, FW_COMMIT_NR };

const char STR_REVISION_DEV  [] = "dev";
const char STR_REVISION_ALPHA[] = "alpha";
const char STR_REVISION_BETA [] = "beta";
const char STR_REVISION_RC   [] = "rc";


inline bool is_whitespace_or_nl(char c)
{
    return c == ' ' || c == '\t' || c == '\n' || c == '\r';
}

inline bool is_whitespace_or_nl_or_eol(char c)
{
    return c == 0 || c == ' ' || c == '\t' || c == '\n' || c == '\r';
}

inline bool is_digit(char c)
{
    return c >= '0' && c <= '9';
}

// Parse a major.minor.revision version number.
// Return true if valid.
inline bool parse_version(const char *str, uint16_t version[4])
{   
#if 0
    ;//SERIAL_ECHOPGM("Parsing version string ");
    ;//SERIAL_ECHO(str);
    ;//SERIAL_ECHOLNPGM("");
#endif

    const char *major = str;
    const char *p = str;
    while (is_digit(*p)) ++ p;
    if (*p != '.')
        return false;
    const char *minor = ++ p;
    while (is_digit(*p)) ++ p;
    if (*p != '.')
        return false;
    const char *rev = ++ p;
    while (is_digit(*p)) ++ p;
    if (! is_whitespace_or_nl_or_eol(*p) && *p != '-')
        return false;

    char *endptr = NULL;
    version[0] = strtol(major, &endptr, 10);
    if (endptr != minor - 1)
        return false;
    version[1] = strtol(minor, &endptr, 10);
    if (endptr != rev - 1)
        return false;
    version[2] = strtol(rev, &endptr, 10);
    if (endptr != p)
        return false;

    version[3] = FIRMWARE_REVISION_RELEASED;
    if (*p ++ == '-') {
        const char *q = p;
        while (! is_whitespace_or_nl_or_eol(*q))
            ++ q;
        uint8_t n = q - p;
        if (n == strlen(STR_REVISION_DEV) && strncmp(p, STR_REVISION_DEV, n) == 0)
            version[3] = FIRMWARE_REVISION_DEV;
        else if (n == strlen(STR_REVISION_ALPHA) && strncmp(p, STR_REVISION_ALPHA, n) == 0)
            version[3] = FIRMWARE_REVISION_ALPHA;
        else if (n == strlen(STR_REVISION_BETA) && strncmp(p, STR_REVISION_BETA, n) == 0)
            version[3] = FIRMWARE_REVISION_BETA;
        else if ((n == 2 || n == 3) && (p[0] == 'r' || p[0] == 'R') && (p[1] == 'c' || p[1] == 'C')) {
            if (n == 2)
                version[3] = FIRMWARE_REVISION_RC;
            else {
                if (is_digit(p[2]))
                    version[3] = FIRMWARE_REVISION_RC + p[2] - '1';
                else
                    return false;
            }
        } else
            return false;
    }

#if 0
    ;//SERIAL_ECHOPGM("Version parsed, major: ");
    ;//SERIAL_ECHO(version[0]);
    ;//SERIAL_ECHOPGM(", minor: ");
    ;//SERIAL_ECHO(version[1]);
    ;//SERIAL_ECHOPGM(", revision: ");
    ;//SERIAL_ECHO(version[2]);
    ;//SERIAL_ECHOPGM(", flavor: ");
    ;//SERIAL_ECHO(version[3]);
    ;//SERIAL_ECHOLNPGM("");
#endif
    return true;
}

inline int8_t is_provided_version_newer(const char *version_string)
{
    uint16_t ver_gcode[4];
    if (! parse_version(version_string, ver_gcode))
        return -1;
    for (uint8_t i = 0; i < 4; ++ i)
    {
        uint16_t v = (uint16_t)FW_VERSION_NR[i];
        if (ver_gcode[i] > v)
            return 1;
        else if (ver_gcode[i] < v)
            return 0;
    }
    return 0;
}

bool show_upgrade_dialog_if_version_newer(const char *version_string)
{
    int8_t upgrade = is_provided_version_newer(version_string);
    if (upgrade < 0)
        return false;

    /* if (upgrade) {
        lcd_display_message_fullscreen_P(_i("New firmware version available:"));////MSG_NEW_FIRMWARE_AVAILABLE c=20 r=2
        lcd_puts_at_P(0, 2, PSTR(""));
        for (const char *c = version_string; ! is_whitespace_or_nl_or_eol(*c); ++ c)
            lcd_putc(*c);
        lcd_puts_at_P(0, 3, _i("Please upgrade."));////MSG_NEW_FIRMWARE_PLEASE_UPGRADE c=20
        Sound_MakeCustom(50,1000,false);
        delay_keep_alive(500);
        Sound_MakeCustom(50,1000,false);
        lcd_wait_for_click_delay(30);
        lcd_update_enable(true);
        lcd_clear();
        lcd_update(0);
    } */

    // Succeeded.
    return true;
}

void fw_version_check(const char *pVersion)
{
uint16_t aVersion[4];
uint8_t nCompareValueResult;

if(oCheckVersion==ClCheckVersion::_None)
     return;
parse_version(pVersion,aVersion);
// nCompareValueResult=mCompareValue(aVersion[0],eeprom_read_word((uint16_t*)EEPROM_FIRMWARE_VERSION_MAJOR))<<6;
// nCompareValueResult+=mCompareValue(aVersion[1],eeprom_read_word((uint16_t*)EEPROM_FIRMWARE_VERSION_MINOR))<<4;
// nCompareValueResult+=mCompareValue(aVersion[2],eeprom_read_word((uint16_t*)EEPROM_FIRMWARE_VERSION_REVISION))<<2;
// nCompareValueResult+=mCompareValue(aVersion[3],eeprom_read_word((uint16_t*)EEPROM_FIRMWARE_VERSION_FLAVOR));
// if(nCompareValueResult==COMPARE_VALUE_EQUAL)
//      return;
// if((nCompareValueResult<COMPARE_VALUE_EQUAL)&&oCheckVersion==ClCheckVersion::_Warn)
//      return;
//SERIAL_ECHO_START;
//SERIAL_ECHOLNPGM("Printer FW version differs from the G-code ...");
//SERIAL_ECHOPGM("actual  : ");
//SERIAL_ECHOLN(FW_VERSION);
//SERIAL_ECHOPGM("expected: ");
//SERIAL_ECHOLN(pVersion);
/* switch(oCheckVersion)
     {
     case ClCheckVersion::_Warn:
//          lcd_show_fullscreen_message_and_wait_P(_i("Printer FW version differs from the G-code. Continue?"));
lcd_display_message_fullscreen_P(_i("G-code sliced for a newer firmware. Continue?"));////MSG_GCODE_NEWER_FIRMWARE_CONTINUE c=20 r=5
lcd_wait_for_click_delay(MSG_PRINT_CHECKING_FAILED_TIMEOUT);
//???custom_message_type=CUSTOM_MSG_TYPE_STATUS; // display / status-line recovery
lcd_update_enable(true);           // display / status-line recovery
          break;
     case ClCheckVersion::_Strict:
          lcd_show_fullscreen_message_and_wait_P(_i("G-code sliced for a newer firmware. Please update the firmware. Print cancelled."));////MSG_GCODE_NEWER_FIRMWARE_CANCELLED c=20 r=8
          lcd_print_stop();
          break;
     case ClCheckVersion::_None:
     case ClCheckVersion::_Undef:
          break;
     } */
}



// G92 - Set current position to coordinates given
static void gcode_G92()
{
    bool codes[NUM_AXIS];
    float values[NUM_AXIS];

    // Check which axes need to be set
    for(uint8_t i = 0; i < NUM_AXIS; ++i)
    {
        codes[i] = code_seen(axis_codes[i]);
        if(codes[i])
            values[i] = code_value();
    }

    if((codes[E_AXIS] && values[E_AXIS] == 0) &&
       (!codes[X_AXIS] && !codes[Y_AXIS] && !codes[Z_AXIS]))
    {
        // As a special optimization, when _just_ clearing the E position
        // we schedule a flag asynchronously along with the next block to
        // reset the starting E position instead of stopping the planner
        current_position[E_AXIS] = 0;
        ;//plan_reset_next_e();
    }
    else
    {
        // In any other case we're forced to synchronize
        ;//st_synchronize();
        for(uint8_t i = 0; i < 3; ++i)
        {
            if(codes[i])
                current_position[i] = values[i] + cs.add_homing[i];
        }
        if(codes[E_AXIS])
            current_position[E_AXIS] = values[E_AXIS];

        // Set all at once
        ;//plan_set_position_curposXYZE();
    }
}

#ifdef TMC2130
void gcode_G28(bool home_x_axis, long home_x_value, bool home_y_axis, long home_y_value, bool home_z_axis, long home_z_value, bool calib, bool without_mbl)
#else
void gcode_G28(bool home_x_axis, long home_x_value, bool home_y_axis, long home_y_value, bool home_z_axis, long home_z_value, bool without_mbl)
#endif //TMC2130
{
	// Flag for the display update routine and to disable the print cancelation during homing.
	;//st_synchronize();
	homing_flag = true;

#if 0
	;//;//SERIAL_ECHOPGM("G28, initial ");  print_world_coordinates();
	;//;//SERIAL_ECHOPGM("G28, initial ");  print_physical_coordinates();
#endif

	// Which axes should be homed?
	bool home_x = home_x_axis;
	bool home_y = home_y_axis;
	bool home_z = home_z_axis;

	// Either all X,Y,Z codes are present, or none of them.
	bool home_all_axes = home_x == home_y && home_x == home_z;
	if (home_all_axes)
		// No X/Y/Z code provided means to home all axes.
		home_x = home_y = home_z = true;

	//if we are homing all axes, first move z higher to protect heatbed/steel sheet
	if (home_all_axes) {
        ;//raise_z_above(MESH_HOME_Z_SEARCH);
		;//st_synchronize();
	}
#ifdef ENABLE_AUTO_BED_LEVELING
      plan_bed_level_matrix.set_to_identity();  //Reset the plane ("erase" all leveling data)
#endif //ENABLE_AUTO_BED_LEVELING
            
      // Reset world2machine_rotation_and_skew and world2machine_shift, therefore
      // the planner will not perform any adjustments in the XY plane. 
      // Wait for the motors to stop and update the current position with the absolute values.
      ;//world2machine_revert_to_uncorrected();

      // For mesh bed leveling deactivate the matrix temporarily.
      // It is necessary to disable the bed leveling for the X and Y homing moves, so that the move is performed
      // in a single axis only.
      // In case of re-homing the X or Y axes only, the mesh bed leveling is restored after G28.
#ifdef MESH_BED_LEVELING
      uint8_t mbl_was_active = mbl.active;
      mbl.active = 0;
      current_position[Z_AXIS] = st_get_position_mm(Z_AXIS);
#endif

      // Reset baby stepping to zero, if the babystepping has already been loaded before.
      if (home_z)
        ;//babystep_undo();

      ;//int l_feedmultiply = setup_for_endstop_move();

      set_destination_to_current();
      feedrate = 0.0;

      #if Z_HOME_DIR > 0                      // If homing away from BED do Z first
      if(home_z)
        homeaxis(Z_AXIS);
      #endif

      #ifdef QUICK_HOME
      // In the quick mode, if both x and y are to be homed, a diagonal move will be performed initially.
      if(home_x && home_y)  //first diagonal move
      {
        current_position[X_AXIS] = 0;current_position[Y_AXIS] = 0;

        uint8_t x_axis_home_dir = home_dir(X_AXIS);

        plan_set_position_curposXYZE();
        destination[X_AXIS] = 1.5 * max_length(X_AXIS) * x_axis_home_dir;destination[Y_AXIS] = 1.5 * max_length(Y_AXIS) * home_dir(Y_AXIS);
        feedrate = homing_feedrate[X_AXIS];
        if(homing_feedrate[Y_AXIS]<feedrate)
          feedrate = homing_feedrate[Y_AXIS];
        if (max_length(X_AXIS) > max_length(Y_AXIS)) {
          feedrate *= sqrt(pow(max_length(Y_AXIS) / max_length(X_AXIS), 2) + 1);
        } else {
          feedrate *= sqrt(pow(max_length(X_AXIS) / max_length(Y_AXIS), 2) + 1);
        }
        plan_buffer_line_destinationXYZE(feedrate/60);
        st_synchronize();

        axis_is_at_home(X_AXIS);
        axis_is_at_home(Y_AXIS);
        plan_set_position_curposXYZE();
        destination[X_AXIS] = current_position[X_AXIS];
        destination[Y_AXIS] = current_position[Y_AXIS];
        plan_buffer_line_destinationXYZE(feedrate/60);
        feedrate = 0.0;
        st_synchronize();
        endstops_hit_on_purpose();

        current_position[X_AXIS] = destination[X_AXIS];
        current_position[Y_AXIS] = destination[Y_AXIS];
        current_position[Z_AXIS] = destination[Z_AXIS];
      }
      #endif /* QUICK_HOME */

#ifdef TMC2130	 
      if(home_x)
	  {
		if (!calib)
			homeaxis(X_AXIS);
		else
			tmc2130_home_calibrate(X_AXIS);
	  }

      if(home_y)
	  {
		if (!calib)
	        homeaxis(Y_AXIS);
		else
			tmc2130_home_calibrate(Y_AXIS);
	  }
#else //TMC2130
      if(home_x) //homeaxis(X_AXIS);
      ;
      if(home_y) 
      ;//homeaxis(Y_AXIS);
#endif //TMC2130


      if(home_x_axis && home_x_value != 0)
        current_position[X_AXIS]=home_x_value+cs.add_homing[X_AXIS];

      if(home_y_axis && home_y_value != 0)
        current_position[Y_AXIS]=home_y_value+cs.add_homing[Y_AXIS];

      #if Z_HOME_DIR < 0                      // If homing towards BED do Z last
        #ifndef Z_SAFE_HOMING
          if(home_z) {
            #if defined (Z_RAISE_BEFORE_HOMING) && (Z_RAISE_BEFORE_HOMING > 0)
              raise_z_above(Z_RAISE_BEFORE_HOMING);
              st_synchronize();
            #endif // defined (Z_RAISE_BEFORE_HOMING) && (Z_RAISE_BEFORE_HOMING > 0)
            #ifdef MESH_BED_LEVELING  // If Mesh bed leveling, move X&Y to safe position for home
              ;//raise_z_above(MESH_HOME_Z_SEARCH);
              ;//st_synchronize();
              if (!axis_known_position[X_AXIS]) homeaxis(X_AXIS);
              if (!axis_known_position[Y_AXIS]) homeaxis(Y_AXIS);
              // 1st mesh bed leveling measurement point, corrected.
              world2machine_initialize();
              world2machine(pgm_read_float(bed_ref_points_4), pgm_read_float(bed_ref_points_4+1), destination[X_AXIS], destination[Y_AXIS]);
              world2machine_reset();
              if (destination[Y_AXIS] < Y_MIN_POS)
                  destination[Y_AXIS] = Y_MIN_POS;
              feedrate = homing_feedrate[X_AXIS] / 20;
              enable_endstops(false);
#ifdef DEBUG_BUILD
              ;//;//SERIAL_ECHOLNPGM("plan_set_position()");
              MYSERIAL.println(current_position[X_AXIS]);MYSERIAL.println(current_position[Y_AXIS]);
              MYSERIAL.println(current_position[Z_AXIS]);MYSERIAL.println(current_position[E_AXIS]);
#endif
              plan_set_position_curposXYZE();
#ifdef DEBUG_BUILD
              ;//;//SERIAL_ECHOLNPGM("plan_buffer_line()");
              MYSERIAL.println(destination[X_AXIS]);MYSERIAL.println(destination[Y_AXIS]);
              MYSERIAL.println(destination[Z_AXIS]);MYSERIAL.println(destination[E_AXIS]);
              MYSERIAL.println(feedrate);MYSERIAL.println(active_extruder);
#endif
              plan_buffer_line_destinationXYZE(feedrate);
              st_synchronize();
              current_position[X_AXIS] = destination[X_AXIS];
              current_position[Y_AXIS] = destination[Y_AXIS];
              enable_endstops(true);
              endstops_hit_on_purpose();
              homeaxis(Z_AXIS);
            #else // MESH_BED_LEVELING
              ;//homeaxis(Z_AXIS);
            #endif // MESH_BED_LEVELING
          }
        #else // defined(Z_SAFE_HOMING): Z Safe mode activated.
          if(home_all_axes) {
            destination[X_AXIS] = round(Z_SAFE_HOMING_X_POINT - X_PROBE_OFFSET_FROM_EXTRUDER);
            destination[Y_AXIS] = round(Z_SAFE_HOMING_Y_POINT - Y_PROBE_OFFSET_FROM_EXTRUDER);
            destination[Z_AXIS] = Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS) * (-1);    // Set destination away from bed
            feedrate = XY_TRAVEL_SPEED/60;
            current_position[Z_AXIS] = 0;

            plan_set_position_curposXYZE();
            plan_buffer_line_destinationXYZE(feedrate);
            st_synchronize();
            current_position[X_AXIS] = destination[X_AXIS];
            current_position[Y_AXIS] = destination[Y_AXIS];

            homeaxis(Z_AXIS);
          }
                                                // Let's see if X and Y are homed and probe is inside bed area.
          if(home_z) {
            if ( (axis_known_position[X_AXIS]) && (axis_known_position[Y_AXIS]) \
              && (current_position[X_AXIS]+X_PROBE_OFFSET_FROM_EXTRUDER >= X_MIN_POS) \
              && (current_position[X_AXIS]+X_PROBE_OFFSET_FROM_EXTRUDER <= X_MAX_POS) \
              && (current_position[Y_AXIS]+Y_PROBE_OFFSET_FROM_EXTRUDER >= Y_MIN_POS) \
              && (current_position[Y_AXIS]+Y_PROBE_OFFSET_FROM_EXTRUDER <= Y_MAX_POS)) {

              current_position[Z_AXIS] = 0;
              plan_set_position_curposXYZE();
              destination[Z_AXIS] = Z_RAISE_BEFORE_HOMING * home_dir(Z_AXIS) * (-1);    // Set destination away from bed
              feedrate = max_feedrate[Z_AXIS];
              plan_buffer_line_destinationXYZE(feedrate);
              st_synchronize();

              homeaxis(Z_AXIS);
            } else if (!((axis_known_position[X_AXIS]) && (axis_known_position[Y_AXIS]))) {
                LCD_MESSAGERPGM(MSG_POSITION_UNKNOWN);
                ;//;//SERIAL_ECHO_START;
                ;//;//SERIAL_ECHOLNRPGM(MSG_POSITION_UNKNOWN);
            } else {
                LCD_MESSAGERPGM(MSG_ZPROBE_OUT);
                ;//;//SERIAL_ECHO_START;
                ;//;//SERIAL_ECHOLNRPGM(MSG_ZPROBE_OUT);
            }
          }
        #endif // Z_SAFE_HOMING
      #endif // Z_HOME_DIR < 0

      if(home_z_axis && home_z_value != 0)
        current_position[Z_AXIS]=home_z_value+cs.add_homing[Z_AXIS];
      #ifdef ENABLE_AUTO_BED_LEVELING
        if(home_z)
          current_position[Z_AXIS] += cs.zprobe_zoffset;  //Add Z_Probe offset (the distance is negative)
      #endif
      
      // Set the planner and stepper routine positions.
      // At this point the mesh bed leveling and world2machine corrections are disabled and current_position
      // contains the machine coordinates.
      ;//plan_set_position_curposXYZE();

      ;//clean_up_after_endstop_move(l_feedmultiply);
      ;//endstops_hit_on_purpose();
#ifndef MESH_BED_LEVELING
//-// Oct 2019 :: this part of code is (from) now probably un-compilable
      // If MESH_BED_LEVELING is not active, then it is the original Prusa i3.
      // Offer the user to load the baby step value, which has been adjusted at the previous print session.
      /*if(card.sdprinting && eeprom_read_word((uint16_t *)EEPROM_BABYSTEP_Z))
          lcd_adjust_z();
        */
#endif

    // Load the machine correction matrix
    ;//world2machine_initialize();
    // and correct the current_position XY axes to match the transformed coordinate system.
    ;//world2machine_update_current();

#ifdef MESH_BED_LEVELING
	if (home_x_axis || home_y_axis || without_mbl || home_z_axis)
    {
      if (! home_z && mbl_was_active) {
        // Re-enable the mesh bed leveling if only the X and Y axes were re-homed.
        mbl.active = true;
        // and re-adjust the current logical Z axis with the bed leveling offset applicable at the current XY position.
        current_position[Z_AXIS] -= mbl.get_z(st_get_position_mm(X_AXIS), st_get_position_mm(Y_AXIS));
      }
    }
#endif

	  if (farm_mode) 
    ;//{ prusa_statistics(20); };

      ;//st_synchronize();
	  homing_flag = false;

#if 0
      ;//;//SERIAL_ECHOPGM("G28, final ");  print_world_coordinates();
      ;//;//SERIAL_ECHOPGM("G28, final ");  print_physical_coordinates();
      ;//;//SERIAL_ECHOPGM("G28, final ");  print_mesh_bed_leveling_table();
#endif
}

void gcode_G28(bool home_x_axis, bool home_y_axis, bool home_z_axis)
{
#ifdef TMC2130
    gcode_G28(home_x_axis, 0, home_y_axis, 0, home_z_axis, 0, false, true);
#else
    gcode_G28(home_x_axis, 0, home_y_axis, 0, home_z_axis, 0, true);
#endif //TMC2130
}


long millis() {
    auto now = std::chrono::system_clock::now();
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
    auto value = now_ms.time_since_epoch();
    return value.count();
}

void get_arc_coordinates()
{
#ifdef SF_ARC_FIX
   bool relative_mode_backup = relative_mode;
   relative_mode = true;
#endif
   get_coordinates();
#ifdef SF_ARC_FIX
   relative_mode=relative_mode_backup;
#endif

   if(code_seen('I')) {
     offset[0] = code_value();
   }
   else {
     offset[0] = 0.0;
   }
   if(code_seen('J')) {
     offset[1] = code_value();
   }
   else {
     offset[1] = 0.0;
   }
}

void get_coordinates()
{
  bool seen[4]={false,false,false,false};
  for(int8_t i=0; i < NUM_AXIS; i++) {
    if(code_seen(axis_codes[i]))
    {
      bool relative = axis_relative_modes & (1 << i);
      destination[i] = code_value();
      #if DEBUG_OUTPUT
      std::cout << axis_codes[i] << ": " << destination[i] << " ";
      #endif
      if (i == E_AXIS) {
        float emult = extruder_multiplier[active_extruder];
        if (emult != 1.) {
          if (! relative) {
            destination[i] -= current_position[i];
            relative = true;
          }
          destination[i] *= emult;
        }
      }
      if (relative)
        destination[i] += current_position[i];
      seen[i]=true;
#if MOTHERBOARD == BOARD_RAMBO_MINI_1_0 || MOTHERBOARD == BOARD_RAMBO_MINI_1_3
	  if (i == Z_AXIS /*&& SilentModeMenu == SILENT_MODE_AUTO*/) 
    ;//update_currents();
#endif //MOTHERBOARD == BOARD_RAMBO_MINI_1_0 || MOTHERBOARD == BOARD_RAMBO_MINI_1_3
    }
    else destination[i] = current_position[i]; //Are these else lines really needed?
  }
  if(code_seen('F')) {
    next_feedrate = code_value();
    #if DEBUG_OUTPUT
      std::cout << "F: " << next_feedrate << " ";
    #endif
    if(next_feedrate > 0.0) feedrate = next_feedrate;
	if (!seen[0] && !seen[1] && !seen[2] && seen[3])
	{
//		float e_max_speed = 
//		printf_P(PSTR("E MOVE speed %7.3f\n"), feedrate / 60)
	}
  }
}

void process_commands()
{
#ifdef FANCHECK
    if(fan_check_error == EFCE_DETECTED) {
        fan_check_error = EFCE_REPORTED;
        if (usb_timer.running())
            lcd_pause_usb_print();
        else
            lcd_pause_print();
    }
#endif

	if (!buflen) return; //empty command
  #ifdef FILAMENT_RUNOUT_SUPPORT
    SET_INPUT(FR_SENS);
  #endif

#ifdef CMDBUFFER_DEBUG
  ;//;//SERIAL_ECHOPGM("Processing a GCODE command: ");
  ;//;//SERIAL_ECHO(cmdbuffer+bufindr+CMDHDRSIZE);
  ;//;//SERIAL_ECHOLNPGM("");
  ;//;//SERIAL_ECHOPGM("In cmdqueue: ");
  ;//;//SERIAL_ECHO(buflen);
  ;//;//SERIAL_ECHOLNPGM("");
#endif /* CMDBUFFER_DEBUG */
  
  unsigned long codenum; //throw away variable
  char *starpos = NULL;
#ifdef ENABLE_AUTO_BED_LEVELING
  float x_tmp, y_tmp, z_tmp, real_z;
#endif

  // PRUSA GCODES
  //KEEPALIVE_STATE(IN_HANDLER);
    /*!

    ---------------------------------------------------------------------------------
    ### M117 - Display Message <a href="https://reprap.org/wiki/G-code#M117:_Display_Message">M117: Display Message</a>
    This causes the given message to be shown in the status line on an attached LCD.
    It is processed early as to allow printing messages that contain G, M, N or T.

    ---------------------------------------------------------------------------------
    ### Special internal commands
    These are used by internal functions to process certain actions in the right order. Some of these are also usable by the user.
    They are processed early as the commands are complex (strings).
    These are only available on the MK3(S) as these require TMC2130 drivers:
        - CRASH DETECTED
        - CRASH RECOVER
        - CRASH_CANCEL
        - TMC_SET_WAVE
        - TMC_SET_STEP
        - TMC_SET_CHOP
    */
    if (code_seen_P("M117")) //moved to highest priority place to be able to to print strings which includes "G", "PRUSA" and "^"
    {
        starpos = (strchr(strchr_pointer + 5, '*'));
        if (starpos != NULL)
            *(starpos) = '\0';
        //lcd_setstatus(strchr_pointer + 5);
        //// disabled custom_message_type = CustomMsg::MsgUpdate;
    }

    /*!
    ### M0, M1 - Stop the printer <a href="https://reprap.org/wiki/G-code#M0:_Stop_or_Unconditional_stop">M0: Stop or Unconditional stop</a>
    #### Usage

      M0 [P<ms<] [S<sec>] [string]
      M1 [P<ms>] [S<sec>] [string] 

    #### Parameters
  
    - `P<ms>`  - Expire time, in milliseconds
    - `S<sec>` - Expire time, in seconds
    - `string` - Must for M1 and optional for M0 message to display on the LCD
    */

    else if (code_seen_P("M0") || code_seen_P("M1 ")) {// M0 and M1 - (Un)conditional stop - Wait for user button press on LCD
        char *src = strchr_pointer + 2;
        codenum = 0;
        bool hasP = false, hasS = false;
        if (code_seen('P')) {
            codenum = code_value_long(); // milliseconds to wait
            hasP = codenum > 0;
        }
        if (code_seen('S')) {
            codenum = code_value_long() * 1000; // seconds to wait
            hasS = codenum > 0;
        }
        starpos = strchr(src, '*');
        if (starpos != NULL) *(starpos) = '\0';
        while (*src == ' ') ++src;
        // disabled custom_message_type = CustomMsg::M0Wait;
        if (!hasP && !hasS && *src != '\0') {
            // disabled -- lcd_setstatus(src);
        } else {
            // farmers want to abuse a bug from the previous firmware releases
            // - they need to see the filename on the status screen instead of "Wait for user..."
            // So we won't update the message in farm mode...
            /* disabled
            if( ! farm_mode){ 
                LCD_MESSAGERPGM(_i("Wait for user..."));////MSG_USERWAIT c=20
            } else {
                // disabled custom_message_type = CustomMsg::Status; // let the lcd display the name of the printed G-code file in farm mode
            } */
        }
        //lcd_ignore_click();				//call lcd_ignore_click also for else ???
        //st_synchronize();
        //previous_millis_cmd.start();
        if (codenum > 0 ) {
            
            codenum += millis();  // keep track of when we started waiting
            //KEEPALIVE_STATE(PAUSED_FOR_USER);
            while(millis() < codenum && !false/* disabled----lcd_clicked()*/) {
                //manage_heater();
                //manage_inactivity(true);
                //lcd_update(0);
            }
            //KEEPALIVE_STATE(IN_HANDLER);
            //lcd_ignore_click(false);
        } else {
            //marlin_wait_for_click();
        }
        /* disabled
        if (IS_SD_PRINTING)
            // disabled custom_message_type = CustomMsg::Status;
        else
            LCD_MESSAGERPGM(MSG_WELCOME);
            */
    }

#ifdef TMC2130
	else if (strncmp_P(CMDBUFFER_CURRENT_STRING, PSTR("CRASH_"), 6) == 0)
	{

    // ### CRASH_DETECTED - TMC2130
    // ---------------------------------
	  if(code_seen_P(PSTR("CRASH_DETECTED")))
	  {
		  uint8_t mask = 0;
		  if (code_seen('X')) mask |= X_AXIS_MASK;
		  if (code_seen('Y')) mask |= Y_AXIS_MASK;
		  crashdet_detected(mask);
	  }

    // ### CRASH_RECOVER - TMC2130
    // ----------------------------------
	  else if(code_seen_P(PSTR("CRASH_RECOVER")))
		  crashdet_recover();

    // ### CRASH_CANCEL - TMC2130
    // ----------------------------------
	  else if(code_seen_P(PSTR("CRASH_CANCEL")))
		  crashdet_cancel();
	}
	else if (strncmp_P(CMDBUFFER_CURRENT_STRING, PSTR("TMC_"), 4) == 0)
	{
    
    // ### TMC_SET_WAVE_ 
    // --------------------
		if (strncmp_P(CMDBUFFER_CURRENT_STRING + 4, PSTR("SET_WAVE_"), 9) == 0)
		{
			uint8_t axis = *(CMDBUFFER_CURRENT_STRING + 13);
			axis = (axis == 'E')?3:(axis - 'X');
			if (axis < 4)
			{
				uint8_t fac = (uint8_t)strtol(CMDBUFFER_CURRENT_STRING + 14, NULL, 10);
				tmc2130_set_wave(axis, 247, fac);
			}
		}
    
    // ### TMC_SET_STEP_
    //  ------------------
		else if (strncmp_P(CMDBUFFER_CURRENT_STRING + 4, PSTR("SET_STEP_"), 9) == 0)
		{
			uint8_t axis = *(CMDBUFFER_CURRENT_STRING + 13);
			axis = (axis == 'E')?3:(axis - 'X');
			if (axis < 4)
			{
				uint8_t step = (uint8_t)strtol(CMDBUFFER_CURRENT_STRING + 14, NULL, 10);
				uint16_t res = tmc2130_get_res(axis);
				tmc2130_goto_step(axis, step & (4*res - 1), 2, 1000, res);
			}
		}

    // ### TMC_SET_CHOP_
    //  -------------------
		else if (strncmp_P(CMDBUFFER_CURRENT_STRING + 4, PSTR("SET_CHOP_"), 9) == 0)
		{
			uint8_t axis = *(CMDBUFFER_CURRENT_STRING + 13);
			axis = (axis == 'E')?3:(axis - 'X');
			if (axis < 4)
			{
				uint8_t chop0 = tmc2130_chopper_config[axis].toff;
				uint8_t chop1 = tmc2130_chopper_config[axis].hstr;
				uint8_t chop2 = tmc2130_chopper_config[axis].hend;
				uint8_t chop3 = tmc2130_chopper_config[axis].tbl;
				char* str_end = 0;
				if (CMDBUFFER_CURRENT_STRING[14])
				{
					chop0 = (uint8_t)strtol(CMDBUFFER_CURRENT_STRING + 14, &str_end, 10) & 15;
					if (str_end && *str_end)
					{
						chop1 = (uint8_t)strtol(str_end, &str_end, 10) & 7;
						if (str_end && *str_end)
						{
							chop2 = (uint8_t)strtol(str_end, &str_end, 10) & 15;
							if (str_end && *str_end)
								chop3 = (uint8_t)strtol(str_end, &str_end, 10) & 3;
						}
					}
				}
				tmc2130_chopper_config[axis].toff = chop0;
				tmc2130_chopper_config[axis].hstr = chop1 & 7;
				tmc2130_chopper_config[axis].hend = chop2 & 15;
				tmc2130_chopper_config[axis].tbl = chop3 & 3;
				tmc2130_setup_chopper(axis, tmc2130_mres[axis], tmc2130_current_h[axis], tmc2130_current_r[axis]);
				//printf_P(_N("TMC_SET_CHOP_%c %d %d %d %d\n"), "xyze"[axis], chop0, chop1, chop2, chop3);
			}
		}
	}
#ifdef BACKLASH_X
	else if (strncmp_P(CMDBUFFER_CURRENT_STRING, PSTR("BACKLASH_X"), 10) == 0)
	{
		uint8_t bl = (uint8_t)strtol(CMDBUFFER_CURRENT_STRING + 10, NULL, 10);
		st_backlash_x = bl;
		printf_P(_N("st_backlash_x = %d\n"), st_backlash_x);
	}
#endif //BACKLASH_X
#ifdef BACKLASH_Y
	else if (strncmp_P(CMDBUFFER_CURRENT_STRING, PSTR("BACKLASH_Y"), 10) == 0)
	{
		uint8_t bl = (uint8_t)strtol(CMDBUFFER_CURRENT_STRING + 10, NULL, 10);
		st_backlash_y = bl;
		printf_P(_N("st_backlash_y = %d\n"), st_backlash_y);
	}
#endif //BACKLASH_Y
#endif //TMC2130
  else if(code_seen_P("PRUSA")){ 
    /*!
    ---------------------------------------------------------------------------------
    ### PRUSA - Internal command set <a href="https://reprap.org/wiki/G-code#G98:_Activate_farm_mode">G98: Activate farm mode - Notes</a>
    
    Set of internal PRUSA commands
    #### Usage
         PRUSA [ Ping | PRN | FAN | fn | thx | uvlo | MMURES | RESET | fv | M28 | SN | Fir | Rev | Lang | Lz | FR ]
    
    #### Parameters
      - `Ping` 
      - `PRN` - Prints revision of the printer
      - `FAN` - Prints fan details
      - `fn` - Prints farm no.
      - `thx` 
      - `uvlo` 
      - `MMURES` - Reset MMU
      - `RESET` - (Careful!)
      - `fv`  - ?
      - `M28` 
      - `SN` 
      - `Fir` - Prints firmware version
      - `Rev`- Prints filament size, elelectronics, nozzle type
      - `Lang` - Reset the language
      - `Lz` 
      - `FR` - Full factory reset
      - `nozzle set <diameter>` - set nozzle diameter (farm mode only), e.g. `PRUSA nozzle set 0.4`
      - `nozzle D<diameter>` - check the nozzle diameter (farm mode only), works like M862.1 P, e.g. `PRUSA nozzle D0.4`
      - `nozzle` - prints nozzle diameter (farm mode only), works like M862.1 P, e.g. `PRUSA nozzle`
    */


		if (code_seen_P("Ping")) {  // PRUSA Ping
			if (farm_mode) {
				PingTime = millis();
			}	  
		}
		else if (code_seen_P("PRN")) { // PRUSA PRN
		  ;//printf_P(_N("%u"), status_number);

        } else if( code_seen_P("FANPINTST")){
            ;//gcode_PRUSA_BadRAMBoFanTest();
        }else if (code_seen_P("FAN")) { // PRUSA FAN
			;//printf_P(_N("E0:%d RPM\nPRN0:%d RPM\n"), 60*fan_speed[0], 60*fan_speed[1]);
		}
		else if (code_seen_P("thx")) // PRUSA thx
		{
			no_response = false;
		}	
		else if (code_seen_P("uvlo")) // PRUSA uvlo
		{
               ;//eeprom_update_byte((uint8_t*)EEPROM_UVLO,0); 
               enquecommand_P("M24"); 
		}	
		else if (code_seen_P("MMURES")) // PRUSA MMURES
		{
			;//mmu_reset();
		}
		else if (code_seen_P("RESET")) { // PRUSA RESET
#ifdef WATCHDOG
#if defined(XFLASH) && defined(BOOTAPP)
            boot_app_magic = BOOT_APP_MAGIC;
            boot_app_flags = BOOT_APP_FLG_RUN;
#endif //defined(XFLASH) && defined(BOOTAPP)
            ;//softReset();
#elif defined(BOOTAPP) //this is a safety precaution. This is because the new bootloader turns off the heaters, but the old one doesn't. The watchdog should be used most of the time.
            asm volatile("jmp 0x3E000");
#endif
		}else if (code_seen_P("fv")) { // PRUSA fv
        // get file version
        #ifdef SDSUPPORT
        card.openFileReadFilteredGcode(strchr_pointer + 3,true);
        while (true) {
            uint16_t readByte = card.getFilteredGcodeChar();
            MYSERIAL.write(readByte);
            if (readByte=='\n') {
                break;
            }
        }
        card.closefile();

        #endif // SDSUPPORT

    }
#ifdef PRUSA_M28
	else if (code_seen_P(PSTR("M28"))) { // PRUSA M28
        trace();
        prusa_sd_card_upload = true;
        card.openFileWrite(strchr_pointer+4);

	}
#endif //PRUSA_M28
	else if (code_seen_P("SN")) { // PRUSA SN
        char SN[20];
        ;//eeprom_read_block(SN, (uint8_t*)EEPROM_PRUSA_SN, 20);
        if (SN[19])
            ;//puts_P(PSTR("SN invalid"));
        else
            ;//puts(SN);

	} else if(code_seen_P("Fir")){ // PRUSA Fir

      ;//;//SERIAL_PROTOCOLLN(FW_VERSION_FULL);

    } else if(code_seen_P("Rev")){ // PRUSA Rev

      ;//;//SERIAL_PROTOCOLLN(FILAMENT_SIZE "-" ELECTRONICS "-" NOZZLE_TYPE );

    } else if(code_seen_P("Lang")) { // PRUSA Lang
	  ;//lang_reset();

	} else if(code_seen_P(PSTR("Lz"))) { // PRUSA Lz
      ;//eeprom_update_word(reinterpret_cast<uint16_t *>(&(EEPROM_Sheets_base->s[(eeprom_read_byte(&(EEPROM_Sheets_base->active_sheet)))].z_offset)),0);

    } else if(code_seen_P(PSTR("FR"))) { // PRUSA FR
        // Factory full reset
        ;//factory_reset(0);
    } else if(code_seen_P(PSTR("MBL"))) { // PRUSA MBL
        // Change the MBL status without changing the logical Z position.
        if(code_seen('V')) {
            bool value = code_value_short();
            ;//st_synchronize();
            /*
            if(value != mbl.active) {
                mbl.active = value;
                // Use plan_set_z_position to reset the physical values
                ;//plan_set_z_position(current_position[Z_AXIS]);
            }*/
        }

//-//
/*
    } else if(code_seen("rrr")) {
MYSERIAL.println("=== checking ===");
MYSERIAL.println(eeprom_read_byte((uint8_t*)EEPROM_CHECK_MODE),DEC);
MYSERIAL.println(eeprom_read_byte((uint8_t*)EEPROM_NOZZLE_DIAMETER),DEC);
MYSERIAL.println(eeprom_read_word((uint16_t*)EEPROM_NOZZLE_DIAMETER_uM),DEC);
MYSERIAL.println(farm_mode,DEC);
MYSERIAL.println(eCheckMode,DEC);
    } else if(code_seen("www")) {
MYSERIAL.println("=== @ FF ===");
eeprom_update_byte((uint8_t*)EEPROM_CHECK_MODE,0xFF);
eeprom_update_byte((uint8_t*)EEPROM_NOZZLE_DIAMETER,0xFF);
eeprom_update_word((uint16_t*)EEPROM_NOZZLE_DIAMETER_uM,0xFFFF);
*/
    } else if (code_seen_P(PSTR("nozzle"))) { // PRUSA nozzle
          uint16_t nDiameter;
          if(code_seen('D'))
               {
               nDiameter=(uint16_t)(code_value()*1000.0+0.5); // [,um]
               ;//nozzle_diameter_check(nDiameter);
               }
          else if(code_seen_P(PSTR("set")) && farm_mode)
               {
               strchr_pointer++;                  // skip 1st char (~ 's')
               strchr_pointer++;                  // skip 2nd char (~ 'e')
               nDiameter=(uint16_t)(code_value()*1000.0+0.5); // [,um]
               ;//eeprom_update_byte((uint8_t*)EEPROM_NOZZLE_DIAMETER,(uint8_t)ClNozzleDiameter::_Diameter_Undef); // for correct synchronization after farm-mode exiting
               ;//eeprom_update_word((uint16_t*)EEPROM_NOZZLE_DIAMETER_uM,nDiameter);
               }
          else ;//;//SERIAL_PROTOCOLLN((float)eeprom_read_word((uint16_t*)EEPROM_NOZZLE_DIAMETER_uM)/1000.0);

//-// !!! SupportMenu
/*
// musi byt PRED "PRUSA model"
    } else if (code_seen("smodel")) { //! PRUSA smodel
          size_t nOffset;
// ! -> "l"
          strchr_pointer+=5*sizeof(*strchr_pointer); // skip 1st - 5th char (~ 'smode')
          nOffset=strspn(strchr_pointer+1," \t\n\r\v\f");
          if(*(strchr_pointer+1+nOffset))
               printer_smodel_check(strchr_pointer);
          else ;//SERIAL_PROTOCOLLN(PRINTER_NAME);
    } else if (code_seen("model")) { //! PRUSA model
          uint16_t nPrinterModel;
          strchr_pointer+=4*sizeof(*strchr_pointer); // skip 1st - 4th char (~ 'mode')
          nPrinterModel=(uint16_t)code_value_long();
          if(nPrinterModel!=0)
               printer_model_check(nPrinterModel);
          else ;//SERIAL_PROTOCOLLN(PRINTER_TYPE);
    } else if (code_seen("version")) { //! PRUSA version
          strchr_pointer+=7*sizeof(*strchr_pointer); // skip 1st - 7th char (~ 'version')
          while(*strchr_pointer==' ')             // skip leading spaces
               strchr_pointer++;
          if(*strchr_pointer!=0)
               fw_version_check(strchr_pointer);
          else ;//SERIAL_PROTOCOLLN(FW_VERSION);
    } else if (code_seen("gcode")) { //! PRUSA gcode
          uint16_t nGcodeLevel;
          strchr_pointer+=4*sizeof(*strchr_pointer); // skip 1st - 4th char (~ 'gcod')
          nGcodeLevel=(uint16_t)code_value_long();
          if(nGcodeLevel!=0)
               gcode_level_check(nGcodeLevel);
          else ;//SERIAL_PROTOCOLLN(GCODE_LEVEL);
*/
	}	
    //else if (code_seen('Cal')) {
		//  lcd_calibration();
	  // }

  } 
  // This prevents reading files with "^" in their names.
  // Since it is unclear, if there is some usage of this construct,
  // it will be deprecated in 3.9 alpha a possibly completely removed in the future:
  // else if (code_seen('^')) {
  //  // nothing, this is a version line
  // }
  else if(code_seen('G'))
  {
	gcode_in_progress = code_value_short();
  #if DEBUG_OUTPUT
      std::cout << "G" << gcode_in_progress << " ";
    #endif
//	printf_P(_N("BEGIN G-CODE=%u\n"), gcode_in_progress);
    switch (gcode_in_progress)
    {

    /*!
    ---------------------------------------------------------------------------------
	 # G Codes
	### G0, G1 - Coordinated movement X Y Z E <a href="https://reprap.org/wiki/G-code#G0_.26_G1:_Move">G0 & G1: Move</a> 
	In Prusa Firmware G0 and G1 are the same.
	#### Usage
	
	      G0 [ X | Y | Z | E | F | S ]
		  G1 [ X | Y | Z | E | F | S ]
	
	#### Parameters
	  - `X` - The position to move to on the X axis
	  - `Y` - The position to move to on the Y axis
	  - `Z` - The position to move to on the Z axis
	  - `E` - The amount to extrude between the starting point and ending point
	  - `F` - The feedrate per minute of the move between the starting point and ending point (if supplied)
	  
    */
    case 0: // G0 -> G1
    case 1: // G1
      if(Stopped == false) {

        #ifdef FILAMENT_RUNOUT_SUPPORT
            
            if(READ(FR_SENS)){

                        int feedmultiplyBckp=feedmultiply;
                        float target[4];
                        float lastpos[4];
                        target[X_AXIS]=current_position[X_AXIS];
                        target[Y_AXIS]=current_position[Y_AXIS];
                        target[Z_AXIS]=current_position[Z_AXIS];
                        target[E_AXIS]=current_position[E_AXIS];
                        lastpos[X_AXIS]=current_position[X_AXIS];
                        lastpos[Y_AXIS]=current_position[Y_AXIS];
                        lastpos[Z_AXIS]=current_position[Z_AXIS];
                        lastpos[E_AXIS]=current_position[E_AXIS];
                        //retract by E
                        
                        target[E_AXIS]+= FILAMENTCHANGE_FIRSTRETRACT ;
                        
                        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], 400, active_extruder);


                        target[Z_AXIS]+= FILAMENTCHANGE_ZADD ;

                        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], 300, active_extruder);

                        target[X_AXIS]= FILAMENTCHANGE_XPOS ;
                        
                        target[Y_AXIS]= FILAMENTCHANGE_YPOS ;
                         
                 
                        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], 70, active_extruder);

                        target[E_AXIS]+= FILAMENTCHANGE_FINALRETRACT ;
                          

                        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], 20, active_extruder);

                        //finish moves
                        st_synchronize();
                        //disable extruder steppers so filament can be removed
                        disable_e0();
                        disable_e1();
                        disable_e2();
                        _delay(100);
                        
                        //LCD_ALERTMESSAGEPGM(_T(MSG_FILAMENTCHANGE));
                        uint8_t cnt=0;
                        int counterBeep = 0;
                        lcd_wait_interact();
                        while(!lcd_clicked()){
                          cnt++;
                          manage_heater();
                          manage_inactivity(true);
                          //lcd_update(0);
                          if(cnt==0)
                          {
                          #if BEEPER > 0
                          
                            if (counterBeep== 500){
                              counterBeep = 0;
                              
                            }
                          
                            
                            SET_OUTPUT(BEEPER);
                            if (counterBeep== 0){
if(eSoundMode!=e_SOUND_MODE_SILENT)
                              WRITE(BEEPER,HIGH);
                            }
                            
                            if (counterBeep== 20){
                              WRITE(BEEPER,LOW);
                            }
                            
                            
                            
                          
                            counterBeep++;
                          #else
                          #endif
                          }
                        }
                        
                        WRITE(BEEPER,LOW);
                        
                        target[E_AXIS]+= FILAMENTCHANGE_FIRSTFEED ;
                        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], 20, active_extruder); 
                        
                        
                        target[E_AXIS]+= FILAMENTCHANGE_FINALFEED ;
                        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], 2, active_extruder); 
                        
                 
                        
                        
                        
                        lcd_change_fil_state = 0;
                        lcd_loading_filament();
                        while ((lcd_change_fil_state == 0)||(lcd_change_fil_state != 1)){
                        
                          lcd_change_fil_state = 0;
                          lcd_alright();
                          switch(lcd_change_fil_state){
                          
                             case 2:
                                     target[E_AXIS]+= FILAMENTCHANGE_FIRSTFEED ;
                                     plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], 20, active_extruder); 
                        
                        
                                     target[E_AXIS]+= FILAMENTCHANGE_FINALFEED ;
                                     plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], 2, active_extruder); 
                                      
                                     
                                     lcd_loading_filament();
                                     break;
                             case 3:
                                     target[E_AXIS]+= FILAMENTCHANGE_FINALFEED ;
                                     plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], 2, active_extruder); 
                                     lcd_loading_color();
                                     break;
                                          
                             default:
                                     lcd_change_success();
                                     break;
                          }
                          
                        }
                        

                        
                      target[E_AXIS]+= 5;
                      plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], 2, active_extruder);
                        
                      target[E_AXIS]+= FILAMENTCHANGE_FIRSTRETRACT;
                      plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], 400, active_extruder);
                        

                        //current_position[E_AXIS]=target[E_AXIS]; //the long retract of L is compensated by manual filament feeding
                        //plan_set_e_position(current_position[E_AXIS]);
                        plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], 70, active_extruder); //should do nothing
                        plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], target[Z_AXIS], target[E_AXIS], 70, active_extruder); //move xy back
                        plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], target[E_AXIS], 200, active_extruder); //move z back
                        
                        
                        target[E_AXIS]= target[E_AXIS] - FILAMENTCHANGE_FIRSTRETRACT;
                        
                      
                             
                        plan_buffer_line(lastpos[X_AXIS], lastpos[Y_AXIS], lastpos[Z_AXIS], target[E_AXIS], 5, active_extruder); //final untretract
                        
                        
                        plan_set_e_position(lastpos[E_AXIS]);
                        
                        feedmultiply=feedmultiplyBckp;
                        
                     
                        
                        char cmd[9];

                        sprintf_P(cmd, PSTR("M220 S%i"), feedmultiplyBckp);
                        enquecommand(cmd);

            }



        #endif

            get_coordinates(); // For X Y Z E F

            // When recovering from a previous print move, restore the originally
            // calculated target position on the first USB/SD command. This accounts
            // properly for relative moves
            if ((saved_target[0] != SAVED_TARGET_UNSET) &&
                ((CMDBUFFER_CURRENT_TYPE == CMDBUFFER_CURRENT_TYPE_SDCARD) ||
                 (CMDBUFFER_CURRENT_TYPE == CMDBUFFER_CURRENT_TYPE_USB_WITH_LINENR)))
            {
                memcpy(destination, saved_target, sizeof(destination));
                saved_target[0] = SAVED_TARGET_UNSET;
            }

		if (total_filament_used > ((current_position[E_AXIS] - destination[E_AXIS]) * 100)) { //protection against total_filament_used overflow
			total_filament_used = total_filament_used + ((destination[E_AXIS] - current_position[E_AXIS]) * 100);
		}

#ifdef FWRETRACT
        if(cs.autoretract_enabled) {
            if( !(code_seen('X') || code_seen('Y') || code_seen('Z')) && code_seen('E')) {
                float echange=destination[E_AXIS]-current_position[E_AXIS];
                if((echange<-MIN_RETRACT && !retracted[active_extruder]) || (echange>MIN_RETRACT && retracted[active_extruder])) { //move appears to be an attempt to retract or recover
                    ;//st_synchronize();
                    current_position[E_AXIS] = destination[E_AXIS]; //hide the slicer-generated retract/recover from calculations
                    ;//plan_set_e_position(current_position[E_AXIS]); //AND from the planner
                    ;//retract(!retracted[active_extruder]);
                    return;
                }
            }
        }
#endif //FWRETRACT

        ;//prepare_move();
        //ClearToSend();
      }
      break;

    /*!
	### G2, G3 - Controlled Arc Move <a href="https://reprap.org/wiki/G-code#G2_.26_G3:_Controlled_Arc_Move">G2 & G3: Controlled Arc Move</a>
	
    These commands don't propperly work with MBL enabled. The compensation only happens at the end of the move, so avoid long arcs.
    
	#### Usage
	
	      G2 [ X | Y | I | E | F ] (Clockwise Arc)
		  G3 [ X | Y | I | E | F ] (Counter-Clockwise Arc)
	
	#### Parameters
	  - `X` - The position to move to on the X axis
	  - `Y` - The position to move to on the Y axis
      - 'Z' - The position to move to on the Z axis
	  - `I` - The point in X space from the current X position to maintain a constant distance from
	  - `J` - The point in Y space from the current Y position to maintain a constant distance from
	  - `E` - The amount to extrude between the starting point and ending point
	  - `F` - The feedrate per minute of the move between the starting point and ending point (if supplied)
	
    */
    case 2: 
      if(Stopped == false) {
        get_arc_coordinates();
        ;//prepare_arc_move(true);
      }
      break;
 
    // -------------------------------
    case 3: 
      if(Stopped == false) {
        get_arc_coordinates();
        ;//prepare_arc_move(false);
      }
      break;


    /*!
	### G4 - Dwell <a href="https://reprap.org/wiki/G-code#G4:_Dwell">G4: Dwell</a>
	Pause the machine for a period of time.
	
	#### Usage
	
	    G4 [ P | S ]
	
	#### Parameters
	  - `P` - Time to wait, in milliseconds
	  - `S` - Time to wait, in seconds
	
    */
    case 4: 
      codenum = 0;
      if(code_seen('P')) codenum = code_value(); // milliseconds to wait
      if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait
	  if(codenum != 0) 
    ;//LCD_MESSAGERPGM(_n("Sleep..."));////MSG_DWELL
      ;//st_synchronize();
      /*
      codenum += millis();  // keep track of when we started waiting
      previous_millis_cmd.start();
      while(millis() < codenum) {
        ;//manage_heater();
        ;//manage_inactivity();
        ;//lcd_update(0);
      }
      */
      break;


#ifdef FWRETRACT
    /*!
	### G10 - Retract <a href="https://reprap.org/wiki/G-code#G10:_Retract">G10: Retract</a>
	Retracts filament according to settings of `M207`
    */
    case 10: 
       #if EXTRUDERS > 1
        retracted_swap[active_extruder]=(code_seen('S') && code_value_long() == 1); // checks for swap retract argument
        retract(true,retracted_swap[active_extruder]);
       #else
        ;//retract(true);
       #endif
      break;


    /*!
	### G11 - Retract recover <a href="https://reprap.org/wiki/G-code#G11:_Unretract">G11: Unretract</a>
	Unretracts/recovers filament according to settings of `M208`
    */
    case 11: 
       #if EXTRUDERS > 1
        retract(false,retracted_swap[active_extruder]);
       #else
        ;//retract(false);
       #endif 
      break;
#endif //FWRETRACT


    /*!
	### G21 - Sets Units to Millimters <a href="https://reprap.org/wiki/G-code#G21:_Set_Units_to_Millimeters">G21: Set Units to Millimeters</a>
	Units are in millimeters. Prusa doesn't support inches.
    */
    case 21: 
      break; //Doing nothing. This is just to prevent serial UNKOWN warnings.
    

    /*!
    ### G28 - Home all Axes one at a time <a href="https://reprap.org/wiki/G-code#G28:_Move_to_Origin_.28Home.29">G28: Move to Origin (Home)</a>
    Using `G28` without any parameters will perfom homing of all axes AND mesh bed leveling, while `G28 W` will just home all axes (no mesh bed leveling).
    #### Usage
	
         G28 [ X | Y | Z | W | C ]
    
	#### Parameters
     - `X` - Flag to go back to the X axis origin
     - `Y` - Flag to go back to the Y axis origin
     - `Z` - Flag to go back to the Z axis origin
     - `W` - Suppress mesh bed leveling if `X`, `Y` or `Z` are not provided
     - `C` - Calibrate X and Y origin (home) - Only on MK3/s
	*/
    case 28: 
    {
      long home_x_value = 0;
      long home_y_value = 0;
      long home_z_value = 0;
      // Which axes should be homed?
      bool home_x = code_seen(axis_codes[X_AXIS]);
      home_x_value = code_value_long();
      bool home_y = code_seen(axis_codes[Y_AXIS]);
      home_y_value = code_value_long();
      bool home_z = code_seen(axis_codes[Z_AXIS]);
      home_z_value = code_value_long();
      bool without_mbl = code_seen('W');
      // calibrate?
#ifdef TMC2130
      bool calib = code_seen('C');
      gcode_G28(home_x, home_x_value, home_y, home_y_value, home_z, home_z_value, calib, without_mbl);
#else
      gcode_G28(home_x, home_x_value, home_y, home_y_value, home_z, home_z_value, without_mbl);
#endif //TMC2130
      if ((home_x || home_y || without_mbl || home_z) == false) {
          ;//gcode_G80();
      }
      break;
    }

#ifdef ENABLE_AUTO_BED_LEVELING
    

    /*!
	### G29 - Detailed Z-Probe <a href="https://reprap.org/wiki/G-code#G29:_Detailed_Z-Probe">G29: Detailed Z-Probe</a>
	In Prusa Firmware this G-code is deactivated by default, must be turned on in the source code.
	
	See `G81`
    */
    case 29: 
        {
            #if Z_MIN_PIN == -1
            #error "You must have a Z_MIN endstop in order to enable Auto Bed Leveling feature! Z_MIN_PIN must point to a valid hardware pin."
            #endif

            // Prevent user from running a G29 without first homing in X and Y
            if (! (axis_known_position[X_AXIS] && axis_known_position[Y_AXIS]) )
            {
                LCD_MESSAGERPGM(MSG_POSITION_UNKNOWN);
                ;//;//SERIAL_ECHO_START;
                ;//;//SERIAL_ECHOLNRPGM(MSG_POSITION_UNKNOWN);
                break; // abort G29, since we don't know where we are
            }

            st_synchronize();
            // make sure the bed_level_rotation_matrix is identity or the planner will get it incorectly
            //vector_3 corrected_position = plan_get_position_mm();
            //corrected_position.debug("position before G29");
            plan_bed_level_matrix.set_to_identity();
            vector_3 uncorrected_position = plan_get_position();
            //uncorrected_position.debug("position durring G29");
            current_position[X_AXIS] = uncorrected_position.x;
            current_position[Y_AXIS] = uncorrected_position.y;
            current_position[Z_AXIS] = uncorrected_position.z;
            plan_set_position_curposXYZE();
            int l_feedmultiply = setup_for_endstop_move();

            feedrate = homing_feedrate[Z_AXIS];
#ifdef AUTO_BED_LEVELING_GRID
            // probe at the points of a lattice grid

            int xGridSpacing = (RIGHT_PROBE_BED_POSITION - LEFT_PROBE_BED_POSITION) / (AUTO_BED_LEVELING_GRID_POINTS-1);
            int yGridSpacing = (BACK_PROBE_BED_POSITION - FRONT_PROBE_BED_POSITION) / (AUTO_BED_LEVELING_GRID_POINTS-1);


            // solve the plane equation ax + by + d = z
            // A is the matrix with rows [x y 1] for all the probed points
            // B is the vector of the Z positions
            // the normal vector to the plane is formed by the coefficients of the plane equation in the standard form, which is Vx*x+Vy*y+Vz*z+d = 0
            // so Vx = -a Vy = -b Vz = 1 (we want the vector facing towards positive Z

            // "A" matrix of the linear system of equations
            double eqnAMatrix[AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS*3];
            // "B" vector of Z points
            double eqnBVector[AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS];


            int probePointCounter = 0;
            bool zig = true;

            for (int yProbe=FRONT_PROBE_BED_POSITION; yProbe <= BACK_PROBE_BED_POSITION; yProbe += yGridSpacing)
            {
              int xProbe, xInc;
              if (zig)
              {
                xProbe = LEFT_PROBE_BED_POSITION;
                //xEnd = RIGHT_PROBE_BED_POSITION;
                xInc = xGridSpacing;
                zig = false;
              } else // zag
              {
                xProbe = RIGHT_PROBE_BED_POSITION;
                //xEnd = LEFT_PROBE_BED_POSITION;
                xInc = -xGridSpacing;
                zig = true;
              }

              for (int xCount=0; xCount < AUTO_BED_LEVELING_GRID_POINTS; xCount++)
              {
                float z_before;
                if (probePointCounter == 0)
                {
                  // raise before probing
                  z_before = Z_RAISE_BEFORE_PROBING;
                } else
                {
                  // raise extruder
                  z_before = current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS;
                }

                float measured_z = probe_pt(xProbe, yProbe, z_before);

                eqnBVector[probePointCounter] = measured_z;

                eqnAMatrix[probePointCounter + 0*AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS] = xProbe;
                eqnAMatrix[probePointCounter + 1*AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS] = yProbe;
                eqnAMatrix[probePointCounter + 2*AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS] = 1;
                probePointCounter++;
                xProbe += xInc;
              }
            }
            clean_up_after_endstop_move(l_feedmultiply);

            // solve lsq problem
            double *plane_equation_coefficients = qr_solve(AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS, 3, eqnAMatrix, eqnBVector);

            ;//SERIAL_PROTOCOLPGM("Eqn coefficients: a: ");
            ;//SERIAL_PROTOCOL(plane_equation_coefficients[0]);
            ;//SERIAL_PROTOCOLPGM(" b: ");
            ;//SERIAL_PROTOCOL(plane_equation_coefficients[1]);
            ;//SERIAL_PROTOCOLPGM(" d: ");
            ;//SERIAL_PROTOCOLLN(plane_equation_coefficients[2]);


            set_bed_level_equation_lsq(plane_equation_coefficients);

            free(plane_equation_coefficients);

#else // AUTO_BED_LEVELING_GRID not defined

            // Probe at 3 arbitrary points
            // probe 1
            float z_at_pt_1 = probe_pt(ABL_PROBE_PT_1_X, ABL_PROBE_PT_1_Y, Z_RAISE_BEFORE_PROBING);

            // probe 2
            float z_at_pt_2 = probe_pt(ABL_PROBE_PT_2_X, ABL_PROBE_PT_2_Y, current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS);

            // probe 3
            float z_at_pt_3 = probe_pt(ABL_PROBE_PT_3_X, ABL_PROBE_PT_3_Y, current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS);

            clean_up_after_endstop_move(l_feedmultiply);

            set_bed_level_equation_3pts(z_at_pt_1, z_at_pt_2, z_at_pt_3);


#endif // AUTO_BED_LEVELING_GRID
            st_synchronize();

            // The following code correct the Z height difference from z-probe position and hotend tip position.
            // The Z height on homing is measured by Z-Probe, but the probe is quite far from the hotend.
            // When the bed is uneven, this height must be corrected.
            real_z = float(st_get_position(Z_AXIS))/cs.axis_steps_per_unit[Z_AXIS];  //get the real Z (since the auto bed leveling is already correcting the plane)
            x_tmp = current_position[X_AXIS] + X_PROBE_OFFSET_FROM_EXTRUDER;
            y_tmp = current_position[Y_AXIS] + Y_PROBE_OFFSET_FROM_EXTRUDER;
            z_tmp = current_position[Z_AXIS];

            apply_rotation_xyz(plan_bed_level_matrix, x_tmp, y_tmp, z_tmp);         //Apply the correction sending the probe offset
            current_position[Z_AXIS] = z_tmp - real_z + current_position[Z_AXIS];   //The difference is added to current position and sent to planner.
            plan_set_position_curposXYZE();
        }
        break;
#ifndef Z_PROBE_SLED

    /*!
	### G30 - Single Z Probe <a href="https://reprap.org/wiki/G-code#G30:_Single_Z-Probe">G30: Single Z-Probe</a>
	In Prusa Firmware this G-code is deactivated by default, must be turned on in the source code.
    */
    case 30: 
        {
            st_synchronize();
            // TODO: make sure the bed_level_rotation_matrix is identity or the planner will get set incorectly
            int l_feedmultiply = setup_for_endstop_move();

            feedrate = homing_feedrate[Z_AXIS];

            run_z_probe();
            ;//SERIAL_PROTOCOLPGM(_T(MSG_BED));
            ;//SERIAL_PROTOCOLPGM(" X: ");
            ;//SERIAL_PROTOCOL(current_position[X_AXIS]);
            ;//SERIAL_PROTOCOLPGM(" Y: ");
            ;//SERIAL_PROTOCOL(current_position[Y_AXIS]);
            ;//SERIAL_PROTOCOLPGM(" Z: ");
            ;//SERIAL_PROTOCOL(current_position[Z_AXIS]);
            ;//SERIAL_PROTOCOLPGM("\n");

            clean_up_after_endstop_move(l_feedmultiply);
        }
        break;
#else

    /*!
	### G31 - Dock the sled <a href="https://reprap.org/wiki/G-code#G31:_Dock_Z_Probe_sled">G31: Dock Z Probe sled</a>
	In Prusa Firmware this G-code is deactivated by default, must be turned on in the source code.
    */
    case 31: 
        dock_sled(true);
        break;


    /*!
	### G32 - Undock the sled <a href="https://reprap.org/wiki/G-code#G32:_Undock_Z_Probe_sled">G32: Undock Z Probe sled</a>
	In Prusa Firmware this G-code is deactivated by default, must be turned on in the source code.
    */
    case 32: 
        dock_sled(false);
        break;
#endif // Z_PROBE_SLED
#endif // ENABLE_AUTO_BED_LEVELING
            
#ifdef MESH_BED_LEVELING

    /*!
	### G30 - Single Z Probe <a href="https://reprap.org/wiki/G-code#G30:_Single_Z-Probe">G30: Single Z-Probe</a>
    Sensor must be over the bed.
    The maximum travel distance before an error is triggered is 10mm.
    */
    case 30: 
        {
            st_synchronize();
            homing_flag = true;

            // TODO: make sure the bed_level_rotation_matrix is identity or the planner will get set incorectly
            int l_feedmultiply = setup_for_endstop_move();

            feedrate = homing_feedrate[Z_AXIS];

            find_bed_induction_sensor_point_z(-10.f, 3);

			printf_P(_N("%S X: %.5f Y: %.5f Z: %.5f\n"), _T(MSG_BED), _x, _y, _z);

            clean_up_after_endstop_move(l_feedmultiply);
            homing_flag = false;
        }
        break;
	
  /*!
  ### G75 - Print temperature interpolation <a href="https://reprap.org/wiki/G-code#G75:_Print_temperature_interpolation">G75: Print temperature interpolation</a>
  Show/print PINDA temperature interpolating.
  */
	case 75:
	{
		for (uint8_t i = 40; i <= 110; i++)
			printf_P(_N("%d  %.2f"), i, temp_comp_interpolation(i));
	}
	break;

  /*!
  ### G76 - PINDA probe temperature calibration <a href="https://reprap.org/wiki/G-code#G76:_PINDA_probe_temperature_calibration">G76: PINDA probe temperature calibration</a>
  This G-code is used to calibrate the temperature drift of the PINDA (inductive Sensor).

  The PINDAv2 sensor has a built-in thermistor which has the advantage that the calibration can be done once for all materials.
  
  The Original i3 Prusa MK2/s uses PINDAv1 and this calibration improves the temperature drift, but not as good as the PINDAv2.

  superPINDA sensor has internal temperature compensation and no thermistor output. There is no point of doing temperature calibration in such case.
  If PINDA_THERMISTOR and SUPERPINDA_SUPPORT is defined during compilation, calibration is skipped with serial message "No PINDA thermistor".
  This can be caused also if PINDA thermistor connection is broken or PINDA temperature is lower than PINDA_MINTEMP.

  #### Example
  
  ```
  G76
  
  echo PINDA probe calibration start
  echo start temperature: 35.0°
  echo ...
  echo PINDA temperature -- Z shift (mm): 0.---
  ```
  */
  case 76: 
	{
#ifdef PINDA_THERMISTOR
        if (!has_temperature_compensation())
        {
            ;//;//SERIAL_ECHOLNPGM("No PINDA thermistor");
            break;
        }

        if (calibration_status() >= CALIBRATION_STATUS_XYZ_CALIBRATION) {
            //we need to know accurate position of first calibration point
            //if xyz calibration was not performed yet, interrupt temperature calibration and inform user that xyz cal. is needed
            lcd_show_fullscreen_message_and_wait_P(_i("Please run XYZ calibration first."));
            break;
        }

        if (!(axis_known_position[X_AXIS] && axis_known_position[Y_AXIS] && axis_known_position[Z_AXIS]))
        {
            // We don't know where we are! HOME!
            // Push the commands to the front of the message queue in the reverse order!
            // There shall be always enough space reserved for these commands.
            repeatcommand_front(); // repeat G76 with all its parameters
            enquecommand_front_P(G28W0);
            break;
        }
        lcd_show_fullscreen_message_and_wait_P(_i("Stable ambient temperature 21-26C is needed a rigid stand is required."));////MSG_TEMP_CAL_WARNING c=20 r=4
        bool result = lcd_show_fullscreen_message_yes_no_and_wait_P(_T(MSG_STEEL_SHEET_CHECK), false, false);

        if (result)
        {
            current_position[Z_AXIS] = MESH_HOME_Z_SEARCH;
            plan_buffer_line_curposXYZE(3000 / 60);
            current_position[Z_AXIS] = 50;
            current_position[Y_AXIS] = 180;
            plan_buffer_line_curposXYZE(3000 / 60);
            st_synchronize();
            lcd_show_fullscreen_message_and_wait_P(_T(MSG_REMOVE_STEEL_SHEET));
            current_position[Y_AXIS] = pgm_read_float(bed_ref_points_4 + 1);
            current_position[X_AXIS] = pgm_read_float(bed_ref_points_4);
            plan_buffer_line_curposXYZE(3000 / 60);
            st_synchronize();
            gcode_G28(false, false, true);

        }
        if ((current_temperature_pinda > 35) && (farm_mode == false)) {
            //waiting for PIDNA probe to cool down in case that we are not in farm mode
            current_position[Z_AXIS] = 100;
            plan_buffer_line_curposXYZE(3000 / 60);
            if (lcd_wait_for_pinda(35) == false) { //waiting for PINDA probe to cool, if this takes more then time expected, temp. cal. fails
                lcd_temp_cal_show_result(false);
                break;
            }
        }

        st_synchronize();
        homing_flag = true; // keep homing on to avoid babystepping while the LCD is enabled

        lcd_update_enable(true);
        ;//;//SERIAL_ECHOLNPGM("PINDA probe calibration start");

        float zero_z;
        int z_shift = 0; //unit: steps
        float start_temp = 5 * (int)(current_temperature_pinda / 5);
        if (start_temp < 35) start_temp = 35;
        if (start_temp < current_temperature_pinda) start_temp += 5;
        printf_P(_N("start temperature: %.1f\n"), start_temp);

//			setTargetHotend(200, 0);
        setTargetBed(70 + (start_temp - 30));

        // disabled custom_message_type = CustomMsg::TempCal;
        custom_message_state = 1;
        lcd_setstatuspgm(_T(MSG_TEMP_CALIBRATION));
        current_position[Z_AXIS] = MESH_HOME_Z_SEARCH;
        plan_buffer_line_curposXYZE(3000 / 60);
        current_position[X_AXIS] = PINDA_PREHEAT_X;
        current_position[Y_AXIS] = PINDA_PREHEAT_Y;
        plan_buffer_line_curposXYZE(3000 / 60);
        current_position[Z_AXIS] = PINDA_PREHEAT_Z;
        plan_buffer_line_curposXYZE(3000 / 60);
        st_synchronize();

        while (current_temperature_pinda < start_temp)
        {
            delay_keep_alive(1000);
            serialecho_temperatures();
        }

        eeprom_update_byte((uint8_t*)EEPROM_CALIBRATION_STATUS_PINDA, 0); //invalidate temp. calibration in case that in will be aborted during the calibration process

        current_position[Z_AXIS] = MESH_HOME_Z_SEARCH;
        plan_buffer_line_curposXYZE(3000 / 60);
        current_position[X_AXIS] = pgm_read_float(bed_ref_points_4);
        current_position[Y_AXIS] = pgm_read_float(bed_ref_points_4 + 1);
        plan_buffer_line_curposXYZE(3000 / 60);
        st_synchronize();

        bool find_z_result = find_bed_induction_sensor_point_z(-1.f);
        if (find_z_result == false) {
            lcd_temp_cal_show_result(find_z_result);
            homing_flag = false;
            break;
        }
        zero_z = current_position[Z_AXIS];

        printf_P(_N("\nZERO: %.3f\n"), current_position[Z_AXIS]);

        int i = -1; for (; i < 5; i++)
        {
            float temp = (40 + i * 5);
            printf_P(_N("\nStep: %d/6 (skipped)\nPINDA temperature: %d Z shift (mm):0\n"), i + 2, (40 + i*5));
            if (i >= 0) {
                eeprom_update_word((uint16_t*)EEPROM_PROBE_TEMP_SHIFT + i, z_shift);
            }
            if (start_temp <= temp) break;
        }

        for (i++; i < 5; i++)
        {
            float temp = (40 + i * 5);
            printf_P(_N("\nStep: %d/6\n"), i + 2);
            custom_message_state = i + 2;
            setTargetBed(50 + 10 * (temp - 30) / 5);
//				setTargetHotend(255, 0);
            current_position[Z_AXIS] = MESH_HOME_Z_SEARCH;
            plan_buffer_line_curposXYZE(3000 / 60);
            current_position[X_AXIS] = PINDA_PREHEAT_X;
            current_position[Y_AXIS] = PINDA_PREHEAT_Y;
            plan_buffer_line_curposXYZE(3000 / 60);
            current_position[Z_AXIS] = PINDA_PREHEAT_Z;
            plan_buffer_line_curposXYZE(3000 / 60);
            st_synchronize();
            while (current_temperature_pinda < temp)
            {
                delay_keep_alive(1000);
                serialecho_temperatures();
            }
            current_position[Z_AXIS] = MESH_HOME_Z_SEARCH;
            plan_buffer_line_curposXYZE(3000 / 60);
            current_position[X_AXIS] = pgm_read_float(bed_ref_points_4);
            current_position[Y_AXIS] = pgm_read_float(bed_ref_points_4 + 1);
            plan_buffer_line_curposXYZE(3000 / 60);
            st_synchronize();
            find_z_result = find_bed_induction_sensor_point_z(-1.f);
            if (find_z_result == false) {
                lcd_temp_cal_show_result(find_z_result);
                break;
            }
            z_shift = (int)((current_position[Z_AXIS] - zero_z)*cs.axis_steps_per_unit[Z_AXIS]);

            printf_P(_N("\nPINDA temperature: %.1f Z shift (mm): %.3f"), current_temperature_pinda, current_position[Z_AXIS] - zero_z);

            eeprom_update_word((uint16_t*)EEPROM_PROBE_TEMP_SHIFT + i, z_shift);
        }
        lcd_temp_cal_show_result(true);
        homing_flag = false;

#else //PINDA_THERMISTOR

		setTargetBed(PINDA_MIN_T);
		float zero_z;
		int z_shift = 0; //unit: steps
		int t_c; // temperature

		if (!(axis_known_position[X_AXIS] && axis_known_position[Y_AXIS] && axis_known_position[Z_AXIS])) {
			// We don't know where we are! HOME!
			// Push the commands to the front of the message queue in the reverse order!
			// There shall be always enough space reserved for these commands.
			repeatcommand_front(); // repeat G76 with all its parameters
			enquecommand_front_P(G28W0);
			break;
		}
		puts_P(_N("PINDA probe calibration start"));
		// disabled custom_message_type = CustomMsg::TempCal;
		custom_message_state = 1;
		lcd_setstatuspgm(_T(MSG_TEMP_CALIBRATION));
		current_position[X_AXIS] = PINDA_PREHEAT_X;
		current_position[Y_AXIS] = PINDA_PREHEAT_Y;
		current_position[Z_AXIS] = PINDA_PREHEAT_Z;
		plan_buffer_line_curposXYZE(3000 / 60);
		st_synchronize();
		
		while (fabs(degBed() - PINDA_MIN_T) > 1) {
			delay_keep_alive(1000);
			serialecho_temperatures();
		}
		
		//enquecommand_P(PSTR("M190 S50"));
		for (int i = 0; i < PINDA_HEAT_T; i++) {
			delay_keep_alive(1000);
			serialecho_temperatures();
		}
		eeprom_update_byte((uint8_t*)EEPROM_CALIBRATION_STATUS_PINDA, 0); //invalidate temp. calibration in case that in will be aborted during the calibration process 

		current_position[Z_AXIS] = 5;
		plan_buffer_line_curposXYZE(3000 / 60);

		current_position[X_AXIS] = BED_X0;
		current_position[Y_AXIS] = BED_Y0;
		plan_buffer_line_curposXYZE(3000 / 60);
		st_synchronize();
		
		find_bed_induction_sensor_point_z(-1.f);
		zero_z = current_position[Z_AXIS];

		printf_P(_N("\nZERO: %.3f\n"), current_position[Z_AXIS]);

		for (int i = 0; i<5; i++) {
			printf_P(_N("\nStep: %d/6\n"), i + 2);
			custom_message_state = i + 2;
			t_c = 60 + i * 10;

			setTargetBed(t_c);
			current_position[X_AXIS] = PINDA_PREHEAT_X;
			current_position[Y_AXIS] = PINDA_PREHEAT_Y;
			current_position[Z_AXIS] = PINDA_PREHEAT_Z;
			plan_buffer_line_curposXYZE(3000 / 60);
			st_synchronize();
			while (degBed() < t_c) {
				delay_keep_alive(1000);
				serialecho_temperatures();
			}
			for (int i = 0; i < PINDA_HEAT_T; i++) {
				delay_keep_alive(1000);
				serialecho_temperatures();
			}
			current_position[Z_AXIS] = 5;
			plan_buffer_line_curposXYZE(3000 / 60);
			current_position[X_AXIS] = BED_X0;
			current_position[Y_AXIS] = BED_Y0;
			plan_buffer_line_curposXYZE(3000 / 60);
			st_synchronize();
			find_bed_induction_sensor_point_z(-1.f);
			z_shift = (int)((current_position[Z_AXIS] - zero_z)*cs.axis_steps_per_unit[Z_AXIS]);

			printf_P(_N("\nTemperature: %d  Z shift (mm): %.3f\n"), t_c, current_position[Z_AXIS] - zero_z);

			eeprom_update_word((uint16_t*)EEPROM_PROBE_TEMP_SHIFT + i, z_shift);
			
		
		}
		// disabled custom_message_type = CustomMsg::Status;

		eeprom_update_byte((uint8_t*)EEPROM_CALIBRATION_STATUS_PINDA, 1);
		puts_P(_N("Temperature calibration done."));
			disable_x();
			disable_y();
			disable_z();
			disable_e0();
			disable_e1();
			disable_e2();
			setTargetBed(0); //set bed target temperature back to 0
		lcd_show_fullscreen_message_and_wait_P(_T(MSG_TEMP_CALIBRATION_DONE));
		eeprom_update_byte((unsigned char *)EEPROM_TEMP_CAL_ACTIVE, 1);
		lcd_update_enable(true);
		lcd_update(2);		

		
#endif //PINDA_THERMISTOR
	}
	break;


    /*!
    ### G80 - Mesh-based Z probe <a href="https://reprap.org/wiki/G-code#G80:_Mesh-based_Z_probe">G80: Mesh-based Z probe</a>
    Default 3x3 grid can be changed on MK2.5/s and MK3/s to 7x7 grid.
    #### Usage
	  
          G80 [ N | R | V | L | R | F | B ]
      
	#### Parameters
      - `N` - Number of mesh points on x axis. Default is 3. Valid values are 3 and 7.
      - `R` - Probe retries. Default 3 max. 10
      - `V` - Verbosity level 1=low, 10=mid, 20=high. It only can be used if the firmware has been compiled with SUPPORT_VERBOSITY active.
      
      Using the following parameters enables additional "manual" bed leveling correction. Valid values are -100 microns to 100 microns.
    #### Additional Parameters
      - `L` - Left Bed Level correct value in um.
      - `R` - Right Bed Level correct value in um.
      - `F` - Front Bed Level correct value in um.
      - `B` - Back Bed Level correct value in um.
    */
  
	/*
    * Probes a grid and produces a mesh to compensate for variable bed height
	* The S0 report the points as below
	*  +----> X-axis
	*  |
	*  |
	*  v Y-axis
	*/

	case 80: {
        gcode_G80();
	}
	break;

        /*!
		### G81 - Mesh bed leveling status <a href="https://reprap.org/wiki/G-code#G81:_Mesh_bed_leveling_status">G81: Mesh bed leveling status</a>
		Prints mesh bed leveling status and bed profile if activated.
        */
        case 81:
            if (mbl.active) {
                ;//SERIAL_PROTOCOLPGM("Num X,Y: ");
                ;//SERIAL_PROTOCOL(MESH_NUM_X_POINTS);
                ;//SERIAL_PROTOCOL(',');
                ;//SERIAL_PROTOCOL(MESH_NUM_Y_POINTS);
                ;//SERIAL_PROTOCOLPGM("\nZ search height: ");
                ;//SERIAL_PROTOCOL(MESH_HOME_Z_SEARCH);
                ;//SERIAL_PROTOCOLLNPGM("\nMeasured points:");
                for (uint8_t y = MESH_NUM_Y_POINTS; y-- > 0;) {
                    for (uint8_t x = 0; x < MESH_NUM_X_POINTS; x++) {
                        ;//SERIAL_PROTOCOLPGM("  ");
                        ;//SERIAL_PROTOCOL_F(mbl.z_values[y][x], 5);
                    }
                    ;//SERIAL_PROTOCOLLN();
                }
            }
            else
                ;//SERIAL_PROTOCOLLNPGM("Mesh bed leveling not active.");
            break;
            
#if 0
        /*!
        ### G82: Single Z probe at current location - Not active <a href="https://reprap.org/wiki/G-code#G82:_Single_Z_probe_at_current_location">G82: Single Z probe at current location</a>
        
        WARNING! USE WITH CAUTION! If you'll try to probe where is no leveling pad, nasty things can happen!
		In Prusa Firmware this G-code is deactivated by default, must be turned on in the source code.
		*/
        case 82:
            ;//SERIAL_PROTOCOLLNPGM("Finding bed ");
            int l_feedmultiply = setup_for_endstop_move();
            find_bed_induction_sensor_point_z();
            clean_up_after_endstop_move(l_feedmultiply);
            ;//SERIAL_PROTOCOLPGM("Bed found at: ");
            ;//SERIAL_PROTOCOL_F(current_position[Z_AXIS], 5);
            ;//SERIAL_PROTOCOLPGM("\n");
            break;

        /*!
        ### G83: Babystep in Z and store to EEPROM - Not active <a href="https://reprap.org/wiki/G-code#G83:_Babystep_in_Z_and_store_to_EEPROM">G83: Babystep in Z and store to EEPROM</a>
		In Prusa Firmware this G-code is deactivated by default, must be turned on in the source code.
		*/
        case 83:
        {
            int babystepz = code_seen('S') ? code_value() : 0;
            int BabyPosition = code_seen('P') ? code_value() : 0;
            
            if (babystepz != 0) {
                //FIXME Vojtech: What shall be the index of the axis Z: 3 or 4?
                // Is the axis indexed starting with zero or one?
                if (BabyPosition > 4) {
                    ;//SERIAL_PROTOCOLLNPGM("Index out of bounds");
                }else{
                    // Save it to the eeprom
                    babystepLoadZ = babystepz;
                    eeprom_update_word((uint16_t*)EEPROM_BABYSTEP_Z0 + BabyPosition, babystepLoadZ);
                    // adjust the Z
                    babystepsTodoZadd(babystepLoadZ);
                }
            
            }
            
        }
        break;
        /*!
        ### G84: UNDO Babystep Z (move Z axis back) - Not active <a href="https://reprap.org/wiki/G-code#G84:_UNDO_Babystep_Z_.28move_Z_axis_back.29">G84: UNDO Babystep Z (move Z axis back)</a>
		In Prusa Firmware this G-code is deactivated by default, must be turned on in the source code.
		*/
        case 84:
            babystepsTodoZsubtract(babystepLoadZ);
            // babystepLoadZ = 0;
            break;
            
        /*!
        ### G85: Pick best babystep - Not active <a href="https://reprap.org/wiki/G-code#G85:_Pick_best_babystep">G85: Pick best babystep</a>
		In Prusa Firmware this G-code is deactivated by default, must be turned on in the source code.
		*/
        case 85:
            lcd_pick_babystep();
            break;
#endif
            
        /*!
        ### G86 - Disable babystep correction after home <a href="https://reprap.org/wiki/G-code#G86:_Disable_babystep_correction_after_home">G86: Disable babystep correction after home</a>
        
        This G-code will be performed at the start of a calibration script.
        (Prusa3D specific)
        */
        case 86:
            calibration_status_store(CALIBRATION_STATUS_LIVE_ADJUST);
            break;
           

        /*!
        ### G87 - Enable babystep correction after home <a href="https://reprap.org/wiki/G-code#G87:_Enable_babystep_correction_after_home">G87: Enable babystep correction after home</a>
        
		This G-code will be performed at the end of a calibration script.
        (Prusa3D specific)
        */
        case 87:
			calibration_status_store(CALIBRATION_STATUS_CALIBRATED);
            break;

        /*!
        ### G88 - Reserved <a href="https://reprap.org/wiki/G-code#G88:_Reserved">G88: Reserved</a>
        
        Currently has no effect. 
        */

        // Prusa3D specific: Don't know what it is for, it is in V2Calibration.gcode

		    case 88:
			      break;


#endif  // ENABLE_MESH_BED_LEVELING
            

    /*!
	### G90 - Switch off relative mode <a href="https://reprap.org/wiki/G-code#G90:_Set_to_Absolute_Positioning">G90: Set to Absolute Positioning</a>
	All coordinates from now on are absolute relative to the origin of the machine. E axis is left intact.
    */
    case 90: {
		axis_relative_modes &= ~(X_AXIS_MASK | Y_AXIS_MASK | Z_AXIS_MASK);
    }
    break;

    /*!
	### G91 - Switch on relative mode <a href="https://reprap.org/wiki/G-code#G91:_Set_to_Relative_Positioning">G91: Set to Relative Positioning</a>
    All coordinates from now on are relative to the last position. E axis is left intact.
	*/
    case 91: {
		axis_relative_modes |= X_AXIS_MASK | Y_AXIS_MASK | Z_AXIS_MASK;
    }
    break;

    /*!
	### G92 - Set position <a href="https://reprap.org/wiki/G-code#G92:_Set_Position">G92: Set Position</a>
    
    It is used for setting the current position of each axis. The parameters are always absolute to the origin.
    If a parameter is omitted, that axis will not be affected.
    If `X`, `Y`, or `Z` axis are specified, the move afterwards might stutter because of Mesh Bed Leveling. `E` axis is not affected if the target position is 0 (`G92 E0`).
	A G92 without coordinates will reset all axes to zero on some firmware. This is not the case for Prusa-Firmware!
    
    #### Usage
	
	      G92 [ X | Y | Z | E ]
	
	#### Parameters
	  - `X` - new X axis position
	  - `Y` - new Y axis position
	  - `Z` - new Z axis position
	  - `E` - new extruder position
	
    */
    case 92: {
        gcode_G92();
    }
    break;

    /*!
    ### G98 - Activate farm mode <a href="https://reprap.org/wiki/G-code#G98:_Activate_farm_mode">G98: Activate farm mode</a>
	Enable Prusa-specific Farm functions and g-code.
    See Internal Prusa commands.
    */
	case 98:
		farm_mode = 1;
		PingTime = millis();
		;//eeprom_update_byte((unsigned char *)EEPROM_FARM_MODE, farm_mode);
		;//SilentModeMenu = SILENT_MODE_OFF;
		;//eeprom_update_byte((unsigned char *)EEPROM_SILENT, SilentModeMenu);
		;//fCheckModeInit();                       // alternatively invoke printer reset
		break;

    /*! ### G99 - Deactivate farm mode <a href="https://reprap.org/wiki/G-code#G99:_Deactivate_farm_mode">G99: Deactivate farm mode</a>
 	Disables Prusa-specific Farm functions and g-code.
   */
	case 99:
		farm_mode = 0;
		;//lcd_printer_connected();
		;//eeprom_update_byte((unsigned char *)EEPROM_FARM_MODE, farm_mode);
		;//lcd_update(2);
          ;//fCheckModeInit();                       // alternatively invoke printer reset
		break;
	default:
		;//printf_P(MSG_UNKNOWN_CODE, 'G', cmdbuffer + bufindr + CMDHDRSIZE);
    }
//	printf_P(_N("END G-CODE=%u\n"), gcode_in_progress);
	gcode_in_progress = 0;
  } // end if(code_seen('G'))
  /*!
  ### End of G-Codes
  */

  /*!
  ---------------------------------------------------------------------------------
  # M Commands
  
  */

  else if(code_seen('M'))
  {

	  int index;
	  for (index = 1; *(strchr_pointer + index) == ' ' || *(strchr_pointer + index) == '\t'; index++);
	   
	 /*for (++strchr_pointer; *strchr_pointer == ' ' || *strchr_pointer == '\t'; ++strchr_pointer);*/
	  if (*(strchr_pointer+index) < '0' || *(strchr_pointer+index) > '9') {
		  ;//printf_P(PSTR("Invalid M code: %s\n"), cmdbuffer + bufindr + CMDHDRSIZE);

	  } else
	  {
	  mcode_in_progress = code_value_short();
//	printf_P(_N("BEGIN M-CODE=%u\n"), mcode_in_progress);

    switch(mcode_in_progress)
    {

    /*!
	### M17 - Enable all axes <a href="https://reprap.org/wiki/G-code#M17:_Enable.2FPower_all_stepper_motors">M17: Enable/Power all stepper motors</a>
    */

    case 17:
        ;//LCD_MESSAGERPGM(_i("No move."));////MSG_NO_MOVE c=20
        ;//enable_x();
        ;//enable_y();
        ;//enable_z();
        ;//enable_e0();
        ;//enable_e1();
        ;//enable_e2();
      break;

#ifdef SDSUPPORT

    /*!
	### M20 - SD Card file list <a href="https://reprap.org/wiki/G-code#M20:_List_SD_card">M20: List SD card</a>
    #### Usage
    
        M20 [ L | T ]
    #### Parameters
    - `T` - Report timestamps as well. The value is one uint32_t encoded as hex. Requires host software parsing (Cap:EXTENDED_M20).
    - `L` - Reports long filenames instead of just short filenames. Requires host software parsing (Cap:EXTENDED_M20).
    */
    case 20:
      KEEPALIVE_STATE(NOT_BUSY); // do not send busy messages during listing. Inhibits the output of manage_heater()
      ;//SERIAL_PROTOCOLLNRPGM(_N("Begin file list"));////MSG_BEGIN_FILE_LIST
      card.ls(CardReader::ls_param(code_seen('L'), code_seen('T')));
      ;//SERIAL_PROTOCOLLNRPGM(_N("End file list"));////MSG_END_FILE_LIST
    break;

    /*!
	### M21 - Init SD card <a href="https://reprap.org/wiki/G-code#M21:_Initialize_SD_card">M21: Initialize SD card</a>
    */
    case 21:
      card.initsd();
      break;

    /*!
	### M22 - Release SD card <a href="https://reprap.org/wiki/G-code#M22:_Release_SD_card">M22: Release SD card</a>
    */
    case 22: 
      card.release();
      break;

    /*!
	### M23 - Select file <a href="https://reprap.org/wiki/G-code#M23:_Select_SD_file">M23: Select SD file</a>
    #### Usage
    
        M23 [filename]
    
    */
    case 23: 
      starpos = (strchr(strchr_pointer + 4,'*'));
	  if(starpos!=NULL)
        *(starpos)='\0';
      card.openFileReadFilteredGcode(strchr_pointer + 4, true);
      break;

    /*!
	### M24 - Start SD print <a href="https://reprap.org/wiki/G-code#M24:_Start.2Fresume_SD_print">M24: Start/resume SD print</a>
    */
    case 24:
	  if (isPrintPaused)
          lcd_resume_print();
      else
      {
          if (!card.get_sdpos())
          {
              // A new print has started from scratch, reset stats
              failstats_reset_print();
#ifndef LA_NOCOMPAT
              la10c_reset();
#endif
          }

          card.startFileprint();
          starttime=_millis();
      }
	  break;

    /*!
	### M26 - Set SD index <a href="https://reprap.org/wiki/G-code#M26:_Set_SD_position">M26: Set SD position</a>
    Set position in SD card file to index in bytes.
    This command is expected to be called after M23 and before M24.
    Otherwise effect of this command is undefined.
    #### Usage
	
	      M26 [ S ]
	
	#### Parameters
	  - `S` - Index in bytes
    */
    case 26: 
      if(card.cardOK && code_seen('S')) {
        long index = code_value_long();
        card.setIndex(index);
        // We don't disable interrupt during update of sdpos_atomic
        // as we expect, that SD card print is not active in this moment
        sdpos_atomic = index;
      }
      break;

    /*!
	### M27 - Get SD status <a href="https://reprap.org/wiki/G-code#M27:_Report_SD_print_status">M27: Report SD print status</a>
    #### Usage
	
	      M27 [ P ]
	
	#### Parameters
	  - `P` - Show full SFN path instead of LFN only.
    */
    case 27:
      card.getStatus(code_seen('P'));
      break;

    /*!
	### M28 - Start SD write <a href="https://reprap.org/wiki/G-code#M28:_Begin_write_to_SD_card">M28: Begin write to SD card</a>
    */
    case 28: 
      starpos = (strchr(strchr_pointer + 4,'*'));
      if(starpos != NULL){
        char* npos = strchr(CMDBUFFER_CURRENT_STRING, 'N');
        strchr_pointer = strchr(npos,' ') + 1;
        *(starpos) = '\0';
      }
      card.openFileWrite(strchr_pointer+4);
      break;

    /*! ### M29 - Stop SD write <a href="https://reprap.org/wiki/G-code#M29:_Stop_writing_to_SD_card">M29: Stop writing to SD card</a>
	Stops writing to the SD file signaling the end of the uploaded file. It is processed very early and it's not written to the card.
    */
    case 29:
      //processed in write to file routine above
      //card,saving = false;
      break;

    /*!
	### M30 - Delete file <a href="https://reprap.org/wiki/G-code#M30:_Delete_a_file_on_the_SD_card">M30: Delete a file on the SD card</a>
    #### Usage
    
        M30 [filename]
    
    */
    case 30:
      if (card.cardOK){
        card.closefile();
        starpos = (strchr(strchr_pointer + 4,'*'));
        if(starpos != NULL){
          char* npos = strchr(CMDBUFFER_CURRENT_STRING, 'N');
          strchr_pointer = strchr(npos,' ') + 1;
          *(starpos) = '\0';
        }
        card.removeFile(strchr_pointer + 4);
      }
      break;

    /*!
	### M32 - Select file and start SD print <a href="https://reprap.org/wiki/G-code#M32:_Select_file_and_start_SD_print">M32: Select file and start SD print</a>
	@todo What are the parameters P and S for in M32?
    */
    case 32:
    {
      if(card.sdprinting) {
        st_synchronize();

      }
      starpos = (strchr(strchr_pointer + 4,'*'));

      char* namestartpos = (strchr(strchr_pointer + 4,'!'));   //find ! to indicate filename string start.
      if(namestartpos==NULL)
      {
        namestartpos=strchr_pointer + 4; //default name position, 4 letters after the M
      }
      else
        namestartpos++; //to skip the '!'

      if(starpos!=NULL)
        *(starpos)='\0';

      bool call_procedure=(code_seen('P'));

      if(strchr_pointer>namestartpos)
        call_procedure=false;  //false alert, 'P' found within filename

      if( card.cardOK )
      {
        card.openFileReadFilteredGcode(namestartpos,!call_procedure);
        if(code_seen('S'))
          if(strchr_pointer<namestartpos) //only if "S" is occuring _before_ the filename
            card.setIndex(code_value_long());
        card.startFileprint();
        if(!call_procedure)
        {
            if(!card.get_sdpos())
            {
                // A new print has started from scratch, reset stats
                failstats_reset_print();
#ifndef LA_NOCOMPAT
                la10c_reset();
#endif
            }
            starttime=_millis(); // procedure calls count as normal print time.
        }
      }
    } break;

    /*!
	### M928 - Start SD logging <a href="https://reprap.org/wiki/G-code#M928:_Start_SD_logging">M928: Start SD logging</a>
    #### Usage
    
        M928 [filename]
    
    */
    case 928: 
      starpos = (strchr(strchr_pointer + 5,'*'));
      if(starpos != NULL){
        char* npos = strchr(CMDBUFFER_CURRENT_STRING, 'N');
        strchr_pointer = strchr(npos,' ') + 1;
        *(starpos) = '\0';
      }
      card.openLogFile(strchr_pointer+5);
      break;

#endif //SDSUPPORT

    /*!
	### M31 - Report current print time <a href="https://reprap.org/wiki/G-code#M31:_Output_time_since_last_M109_or_SD_card_start_to_serial">M31: Output time since last M109 or SD card start to serial</a>
    */
    case 31: //M31 take time since the start of the SD print or an M109 command
      {
      stoptime=millis();
      char time[30];
      unsigned long t=(stoptime-starttime)/1000;
      int sec,min;
      min=t/60;
      sec=t%60;
      ;//sprintf_P(time, PSTR("%i min, %i sec"), min, sec);
      ;//;//;//SERIAL_ECHO_START;
      ;//;//;//SERIAL_ECHOLN(time);
      ;//lcd_setstatus(time);
      ;//autotempShutdown();
      }
      break;

    /*!
	### M42 - Set pin state <a href="https://reprap.org/wiki/G-code#M42:_Switch_I.2FO_pin">M42: Switch I/O pin</a>
    #### Usage
    
        M42 [ P | S ]
        
    #### Parameters
    - `P` - Pin number.
    - `S` - Pin value. If the pin is analog, values are from 0 to 255. If the pin is digital, values are from 0 to 1.
    
    */
    case 42:
    {
      if (code_seen('S'))
      {
        uint8_t pin_status = code_value_uint8();
        int8_t pin_number = LED_PIN;
        if (code_seen('P'))
          pin_number = code_value_uint8();
        for(int8_t i = 0; i < (int8_t)(sizeof(sensitive_pins)/sizeof(sensitive_pins[0])); i++)
        {
          if ((int8_t)sensitive_pins[i] == pin_number)
          {
            pin_number = -1;
            break;
          }
        }
      #if defined(FAN_PIN) && FAN_PIN > -1
        if (pin_number == FAN_PIN)
          fanSpeed = pin_status;
      #endif
        if (pin_number > -1)
        {
          ;//pinMode(pin_number, OUTPUT);
          ;//digitalWrite(pin_number, pin_status);
          ;//analogWrite(pin_number, pin_status);
        }
      }
    }
     break;


    /*!
	### M44 - Reset the bed skew and offset calibration <a href="https://reprap.org/wiki/G-code#M44:_Reset_the_bed_skew_and_offset_calibration">M44: Reset the bed skew and offset calibration</a>
    */
    case 44: // M44: Prusa3D: Reset the bed skew and offset calibration.

		// Reset the baby step value and the baby step applied flag.
		;//calibration_status_store(CALIBRATION_STATUS_ASSEMBLED);
          ;//eeprom_update_word(reinterpret_cast<uint16_t *>(&(EEPROM_Sheets_base->s[(eeprom_read_byte(&(EEPROM_Sheets_base->active_sheet)))].z_offset)),0);

        // Reset the skew and offset in both RAM and EEPROM.
        ;//reset_bed_offset_and_skew();
        // Reset world2machine_rotation_and_skew and world2machine_shift, therefore
        // the planner will not perform any adjustments in the XY plane. 
        // Wait for the motors to stop and update the current position with the absolute values.
        ;//world2machine_revert_to_uncorrected();
        break;

    /*!
	### M45 - Bed skew and offset with manual Z up <a href="https://reprap.org/wiki/G-code#M45:_Bed_skew_and_offset_with_manual_Z_up">M45: Bed skew and offset with manual Z up</a>
	#### Usage
    
        M45 [ V ]
    #### Parameters
	- `V` - Verbosity level 1, 10 and 20 (low, mid, high). Only when SUPPORT_VERBOSITY is defined. Optional.
    - `Z` - If it is provided, only Z calibration will run. Otherwise full calibration is executed.
    */
    case 45: // M45: Prusa3D: bed skew and offset with manual Z up
    {
		int8_t verbosity_level = 0;
		bool only_Z = code_seen('Z');
		#ifdef SUPPORT_VERBOSITY
		if (code_seen('V'))
		{
			// Just 'V' without a number counts as V1.
			char c = strchr_pointer[1];
			verbosity_level = (c == ' ' || c == '\t' || c == 0) ? 1 : code_value_short();
		}
		#endif //SUPPORT_VERBOSITY
		;//gcode_M45(only_Z, verbosity_level);
    }
	break;

    /*!
	### M46 - Show the assigned IP address <a href="https://reprap.org/wiki/G-code#M46:_Show_the_assigned_IP_address">M46: Show the assigned IP address.</a>
    */
    case 46:
    {
        // M46: Prusa3D: Show the assigned IP address.
        /*
        if (card.ToshibaFlashAir_isEnabled()) {
            uint8_t ip[4];
            if (card.ToshibaFlashAir_GetIP(ip)) {
                // ;//SERIAL_PROTOCOLPGM("Toshiba FlashAir current IP: ");
                ;//SERIAL_PROTOCOL(uint8_t(ip[0]));
                ;//SERIAL_PROTOCOL('.');
                ;//SERIAL_PROTOCOL(uint8_t(ip[1]));
                ;//SERIAL_PROTOCOL('.');
                ;//SERIAL_PROTOCOL(uint8_t(ip[2]));
                ;//SERIAL_PROTOCOL('.');
                ;//SERIAL_PROTOCOLLN(uint8_t(ip[3]));
            } else {
                ;//SERIAL_PROTOCOLPGM("?Toshiba FlashAir GetIP failed\n");          
            }
        } else {
            ;//SERIAL_PROTOCOLLNPGM("n/a");          
        }
        */
        
    }
    break;

    /*!
	### M47 - Show end stops dialog on the display <a href="https://reprap.org/wiki/G-code#M47:_Show_end_stops_dialog_on_the_display">M47: Show end stops dialog on the display</a>
    */
    case 47:
    /*    
		KEEPALIVE_STATE(PAUSED_FOR_USER);
        lcd_diag_show_end_stops();
		KEEPALIVE_STATE(IN_HANDLER);*/
        break;

#if 0
    case 48: // M48: scan the bed induction sensor points, print the sensor trigger coordinates to the serial line for visualization on the PC.
    {
        // Disable the default update procedure of the display. We will do a modal dialog.
        lcd_update_enable(false);
        // Let the planner use the uncorrected coordinates.
        mbl.reset();
        // Reset world2machine_rotation_and_skew and world2machine_shift, therefore
        // the planner will not perform any adjustments in the XY plane. 
        // Wait for the motors to stop and update the current position with the absolute values.
        world2machine_revert_to_uncorrected();
        // Move the print head close to the bed.
        current_position[Z_AXIS] = MESH_HOME_Z_SEARCH;
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS],current_position[Z_AXIS] , current_position[E_AXIS], homing_feedrate[Z_AXIS]/40, active_extruder);
        st_synchronize();
        // Home in the XY plane.
        set_destination_to_current();
        int l_feedmultiply = setup_for_endstop_move();
        home_xy();
        int8_t verbosity_level = 0;
        if (code_seen('V')) {
            // Just 'V' without a number counts as V1.
            char c = strchr_pointer[1];
            verbosity_level = (c == ' ' || c == '\t' || c == 0) ? 1 : code_value_short();
        }
        bool success = scan_bed_induction_points(verbosity_level);
        clean_up_after_endstop_move(l_feedmultiply);
        // Print head up.
        current_position[Z_AXIS] = MESH_HOME_Z_SEARCH;
        plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS],current_position[Z_AXIS] , current_position[E_AXIS], homing_feedrate[Z_AXIS]/40, active_extruder);
        st_synchronize();
        lcd_update_enable(true);
        break;
    }
#endif


#ifdef ENABLE_AUTO_BED_LEVELING
#ifdef Z_PROBE_REPEATABILITY_TEST 

    /*!
	### M48 - Z-Probe repeatability measurement function <a href="https://reprap.org/wiki/G-code#M48:_Measure_Z-Probe_repeatability">M48: Measure Z-Probe repeatability</a>
    
     This function assumes the bed has been homed.  Specifically, that a G28 command as been issued prior to invoking the M48 Z-Probe repeatability measurement function. Any information generated by a prior G29 Bed leveling command will be lost and needs to be regenerated.
     
     The number of samples will default to 10 if not specified.  You can use upper or lower case letters for any of the options EXCEPT n.  n must be in lower case because Marlin uses a capital N for its communication protocol and will get horribly confused if you send it a capital N.
     @todo Why would you check for both uppercase and lowercase? Seems wasteful.
	 
     #### Usage
     
	     M48 [ n | X | Y | V | L ]
     
     #### Parameters
       - `n` - Number of samples. Valid values 4-50
	   - `X` - X position for samples
	   - `Y` - Y position for samples
	   - `V` - Verbose level. Valid values 1-4
	   - `L` - Legs of movementprior to doing probe. Valid values 1-15
    */
    case 48: // M48 Z-Probe repeatability
        {
            #if Z_MIN_PIN == -1
            #error "You must have a Z_MIN endstop in order to enable calculation of Z-Probe repeatability."
            #endif

	double sum=0.0; 
	double mean=0.0; 
	double sigma=0.0;
	double sample_set[50];
	int verbose_level=1, n=0, j, n_samples = 10, n_legs=0;
	double X_current, Y_current, Z_current;
	double X_probe_location, Y_probe_location, Z_start_location, ext_position;
	
	if (code_seen('V') || code_seengcode_M45('v')) {
        	verbose_level = code_value();
		if (verbose_level<0 || verbose_level>4 ) {
			;//SERIAL_PROTOCOLPGM("?Verbose Level not plausable.\n");
			goto Sigma_Exit;
		}
	}

	if (verbose_level > 0)   {
		;//SERIAL_PROTOCOLPGM("M48 Z-Probe Repeatability test.   Version 2.00\n");
		;//SERIAL_PROTOCOLPGM("Full support at: http://3dprintboard.com/forum.php\n");
	}

	if (code_seen('n')) {
        	n_samples = code_value();
		if (n_samples<4 || n_samples>50 ) {
			;//SERIAL_PROTOCOLPGM("?Specified sample size not plausable.\n");
			goto Sigma_Exit;
		}
	}

	X_current = X_probe_location = st_get_position_mm(X_AXIS);
	Y_current = Y_probe_location = st_get_position_mm(Y_AXIS);
	Z_current = st_get_position_mm(Z_AXIS);
	Z_start_location = st_get_position_mm(Z_AXIS) + Z_RAISE_BEFORE_PROBING;
	ext_position	 = st_get_position_mm(E_AXIS);

	if (code_seen('X') || code_seen('x') ) {
        	X_probe_location = code_value() -  X_PROBE_OFFSET_FROM_EXTRUDER;
		if (X_probe_location<X_MIN_POS || X_probe_location>X_MAX_POS ) {
			;//SERIAL_PROTOCOLPGM("?Specified X position out of range.\n");
			goto Sigma_Exit;
		}
	}

	if (code_seen('Y') || code_seen('y') ) {
        	Y_probe_location = code_value() -  Y_PROBE_OFFSET_FROM_EXTRUDER;
		if (Y_probe_location<Y_MIN_POS || Y_probe_location>Y_MAX_POS ) {
			;//SERIAL_PROTOCOLPGM("?Specified Y position out of range.\n");
			goto Sigma_Exit;
		}
	}

	if (code_seen('L') || code_seen('l') ) {
        	n_legs = code_value();
		if ( n_legs==1 ) 
			n_legs = 2;
		if ( n_legs<0 || n_legs>15 ) {
			;//SERIAL_PROTOCOLPGM("?Specified number of legs in movement not plausable.\n");
			goto Sigma_Exit;
		}
	}

//
// Do all the preliminary setup work.   First raise the probe.
//

        st_synchronize();
        plan_bed_level_matrix.set_to_identity();
	plan_buffer_line( X_current, Y_current, Z_start_location,
			ext_position,
    			homing_feedrate[Z_AXIS]/60,
			active_extruder);
        st_synchronize();

//
// Now get everything to the specified probe point So we can safely do a probe to
// get us close to the bed.  If the Z-Axis is far from the bed, we don't want to 
// use that as a starting point for each probe.
//
	if (verbose_level > 2) 
		;//SERIAL_PROTOCOL("Positioning probe for the test.\n");

	plan_buffer_line( X_probe_location, Y_probe_location, Z_start_location,
			ext_position,
    			homing_feedrate[X_AXIS]/60,
			active_extruder);
        st_synchronize();

	current_position[X_AXIS] = X_current = st_get_position_mm(X_AXIS);
	current_position[Y_AXIS] = Y_current = st_get_position_mm(Y_AXIS);
	current_position[Z_AXIS] = Z_current = st_get_position_mm(Z_AXIS);
	current_position[E_AXIS] = ext_position = st_get_position_mm(E_AXIS);

// 
// OK, do the inital probe to get us close to the bed.
// Then retrace the right amount and use that in subsequent probes
//

	int l_feedmultiply = setup_for_endstop_move();
	run_z_probe();

	current_position[Z_AXIS] = Z_current = st_get_position_mm(Z_AXIS);
	Z_start_location = st_get_position_mm(Z_AXIS) + Z_RAISE_BEFORE_PROBING;

	plan_buffer_line( X_probe_location, Y_probe_location, Z_start_location,
			ext_position,
    			homing_feedrate[X_AXIS]/60,
			active_extruder);
        st_synchronize();
	current_position[Z_AXIS] = Z_current = st_get_position_mm(Z_AXIS);

        for( n=0; n<n_samples; n++) {

		do_blocking_move_to( X_probe_location, Y_probe_location, Z_start_location); // Make sure we are at the probe location

		if ( n_legs)  {
		double radius=0.0, theta=0.0, x_sweep, y_sweep;
		int rotational_direction, l;

			rotational_direction = (unsigned long) _millis() & 0x0001;			// clockwise or counter clockwise
			radius = (unsigned long) _millis() % (long) (X_MAX_LENGTH/4); 			// limit how far out to go 
			theta = (float) ((unsigned long) _millis() % (long) 360) / (360./(2*3.1415926));	// turn into radians

//;//;//SERIAL_ECHOPAIR("starting radius: ",radius);
//;//;//SERIAL_ECHOPAIR("   theta: ",theta);
//;//;//SERIAL_ECHOPAIR("   direction: ",rotational_direction);
//;//SERIAL_PROTOCOLLNPGM("");

			for( l=0; l<n_legs-1; l++) {
				if (rotational_direction==1)
					theta += (float) ((unsigned long) _millis() % (long) 20) / (360.0/(2*3.1415926)); // turn into radians
				else
					theta -= (float) ((unsigned long) _millis() % (long) 20) / (360.0/(2*3.1415926)); // turn into radians

				radius += (float) ( ((long) ((unsigned long) _millis() % (long) 10)) - 5);
				if ( radius<0.0 )
					radius = -radius;

				X_current = X_probe_location + cos(theta) * radius;
				Y_current = Y_probe_location + sin(theta) * radius;

				if ( X_current<X_MIN_POS)		// Make sure our X & Y are sane
					 X_current = X_MIN_POS;
				if ( X_current>X_MAX_POS)
					 X_current = X_MAX_POS;

				if ( Y_current<Y_MIN_POS)		// Make sure our X & Y are sane
					 Y_current = Y_MIN_POS;
				if ( Y_current>Y_MAX_POS)
					 Y_current = Y_MAX_POS;

				if (verbose_level>3 ) {
					;//;//SERIAL_ECHOPAIR("x: ", X_current);
					;//;//SERIAL_ECHOPAIR("y: ", Y_current);
					;//SERIAL_PROTOCOLLNPGM("");
				}

				do_blocking_move_to( X_current, Y_current, Z_current );
			}
			do_blocking_move_to( X_probe_location, Y_probe_location, Z_start_location); // Go back to the probe location
		}

		int l_feedmultiply = setup_for_endstop_move();
                run_z_probe();

		sample_set[n] = current_position[Z_AXIS];

//
// Get the current mean for the data points we have so far
//
		sum=0.0; 
		for( j=0; j<=n; j++) {
			sum = sum + sample_set[j];
		}
		mean = sum / (double (n+1));
//
// Now, use that mean to calculate the standard deviation for the
// data points we have so far
//

		sum=0.0; 
		for( j=0; j<=n; j++) {
			sum = sum + (sample_set[j]-mean) * (sample_set[j]-mean);
		}
		sigma = sqrt( sum / (double (n+1)) );

		if (verbose_level > 1) {
			;//SERIAL_PROTOCOL(n+1);
			;//SERIAL_PROTOCOL(" of ");
			;//SERIAL_PROTOCOL(n_samples);
			;//SERIAL_PROTOCOLPGM("   z: ");
			;//SERIAL_PROTOCOL_F(current_position[Z_AXIS], 6);
		}

		if (verbose_level > 2) {
			;//SERIAL_PROTOCOL(" mean: ");
			;//SERIAL_PROTOCOL_F(mean,6);

			;//SERIAL_PROTOCOL("   sigma: ");
			;//SERIAL_PROTOCOL_F(sigma,6);
		}

		if (verbose_level > 0) 
			;//SERIAL_PROTOCOLPGM("\n");

		plan_buffer_line( X_probe_location, Y_probe_location, Z_start_location, 
				  current_position[E_AXIS], homing_feedrate[Z_AXIS]/60, active_extruder);
        	st_synchronize();

	}

	_delay(1000);

    clean_up_after_endstop_move(l_feedmultiply);

//  enable_endstops(true);

	if (verbose_level > 0) {
		;//SERIAL_PROTOCOLPGM("Mean: ");
		;//SERIAL_PROTOCOL_F(mean, 6);
		;//SERIAL_PROTOCOLPGM("\n");
	}

;//SERIAL_PROTOCOLPGM("Standard Deviation: ");
;//SERIAL_PROTOCOL_F(sigma, 6);
;//SERIAL_PROTOCOLPGM("\n\n");

Sigma_Exit:
        break;
	}
#endif		// Z_PROBE_REPEATABILITY_TEST 
#endif		// ENABLE_AUTO_BED_LEVELING

    /*!
    ### M73 - Set/get print progress <a href="https://reprap.org/wiki/G-code#M73:_Set.2FGet_build_percentage">M73: Set/Get build percentage</a>
    #### Usage
    
        M73 [ P | R | Q | S | C | D ]

    #### Parameters
        - `P` - Percent in normal mode
        - `R` - Time remaining in normal mode
        - `Q` - Percent in silent mode
        - `S` - Time in silent mode
        - `C` - Time to change/pause/user interaction in normal mode
        - `D` - Time to change/pause/user interaction in silent mode
    */
    case 73: //M73 show percent done, time remaining and time to change/pause
    {
        if(code_seen('P')) print_percent_done_normal = code_value_uint8();
        if(code_seen('R')) print_time_remaining_normal = code_value();
        if(code_seen('Q')) print_percent_done_silent = code_value_uint8();
        if(code_seen('S')) print_time_remaining_silent = code_value();
        if(code_seen('C')){
            float print_time_to_change_normal_f = code_value_float();
            print_time_to_change_normal = ( print_time_to_change_normal_f <= 0 ) ? PRINT_TIME_REMAINING_INIT : print_time_to_change_normal_f;
        }
        if(code_seen('D')){
            float print_time_to_change_silent_f = code_value_float();
            print_time_to_change_silent = ( print_time_to_change_silent_f <= 0 ) ? PRINT_TIME_REMAINING_INIT : print_time_to_change_silent_f;
        }
        {
            const char* _msg_mode_done_remain = _N("%S MODE: Percent done: %hhd; print time remaining in mins: %d; Change in mins: %d\n");
            ;//printf_P(_msg_mode_done_remain, _N("NORMAL"), int8_t(print_percent_done_normal), print_time_remaining_normal, print_time_to_change_normal);
            ;//printf_P(_msg_mode_done_remain, _N("SILENT"), int8_t(print_percent_done_silent), print_time_remaining_silent, print_time_to_change_silent);
        } 
    }
    break;
    /*!
	### M104 - Set hotend temperature <a href="https://reprap.org/wiki/G-code#M104:_Set_Extruder_Temperature">M104: Set Extruder Temperature</a>
	#### Usage
    
	    M104 [ S ]
    
	#### Parameters
       - `S` - Target temperature
    */
    case 104: // M104
    {
          uint8_t extruder;
          /*
          if(setTargetedHotend(104,extruder)){
            break;
          }
          if (code_seen('S'))
          {
              setTargetHotendSafe(code_value(), extruder);
          }
          */
          break;
    }

    /*!
	### M112 - Emergency stop <a href="https://reprap.org/wiki/G-code#M112:_Full_.28Emergency.29_Stop">M112: Full (Emergency) Stop</a>
    It is processed much earlier as to bypass the cmdqueue.
    */
    case 112: 
    {
      ;//kill(MSG_M112_KILL, 3);
    }
      break;

    /*!
	### M140 - Set bed temperature <a href="https://reprap.org/wiki/G-code#M140:_Set_Bed_Temperature_.28Fast.29">M140: Set Bed Temperature (Fast)</a>
    #### Usage
    
	    M140 [ S ]
    
	#### Parameters
       - `S` - Target temperature
    */
    case 140: 
    {
      if (code_seen('S')) /*setTargetBed(*/code_value();//);
    }
      break;

    /*!
	### M105 - Report temperatures <a href="https://reprap.org/wiki/G-code#M105:_Get_Extruder_Temperature">M105: Get Extruder Temperature</a>
	Prints temperatures:
	
	  - `T:`  - Hotend (actual / target)
	  - `B:`  - Bed (actual / target)
	  - `Tx:` - x Tool (actual / target)
	  - `@:`  - Hotend power
	  - `B@:` - Bed power
	  - `P:`  - PINDAv2 actual (only MK2.5/s and MK3/s)
	  - `A:`  - Ambient actual (only MK3/s)
	
	_Example:_
	
	    ok T:20.2 /0.0 B:19.1 /0.0 T0:20.2 /0.0 @:0 B@:0 P:19.8 A:26.4
	
    */
    case 105:
    {
      uint8_t extruder;
      /*
      if(setTargetedHotend(105, extruder)){
        break;
      }
      
      ;//SERIAL_PROTOCOLPGM("ok ");
      gcode_M105(extruder);*/
      
      cmdqueue_pop_front(); //prevent an ok after the command since this command uses an ok at the beginning.
      cmdbuffer_front_already_processed = true;
      
      break;
    }

#if defined(AUTO_REPORT)
    /*!
	### M155 - Automatically send status <a href="https://reprap.org/wiki/G-code#M155:_Automatically_send_temperatures">M155: Automatically send temperatures</a>
	#### Usage
	
		M155 [ S ] [ C ]
	
	#### Parameters
	
	- `S` - Set autoreporting interval in seconds. 0 to disable. Maximum: 255
	- `C` - Activate auto-report function (bit mask). Default is temperature.

          bit 0 = Auto-report temperatures
          bit 1 = Auto-report fans
          bit 2 = Auto-report position
          bit 3 = free
          bit 4 = free
          bit 5 = free
          bit 6 = free
          bit 7 = free
     */
    case 155:
    {
        if (code_seen('S')){
            ;//autoReportFeatures.SetPeriod( code_value_uint8() );
        }
        if (code_seen('C')){
            ;//autoReportFeatures.SetMask(code_value_uint8());
        } else{
            ;//autoReportFeatures.SetMask(1); //Backwards compability to host systems like Octoprint to send only temp if paramerter `C`isn't used.
        }
   }
    break;
#endif //AUTO_REPORT

    /*!
	### M109 - Wait for extruder temperature <a href="https://reprap.org/wiki/G-code#M109:_Set_Extruder_Temperature_and_Wait">M109: Set Extruder Temperature and Wait</a>
    #### Usage
    
	    M104 [ B | R | S ]
    
    #### Parameters (not mandatory)
     
	  - `S` - Set extruder temperature
      - `R` - Set extruder temperature
	  - `B` - Set max. extruder temperature, while `S` is min. temperature. Not active in default, only if AUTOTEMP is defined in source code.
    
    Parameters S and R are treated identically.
    Command always waits for both cool down and heat up.
    If no parameters are supplied waits for previously set extruder temperature.
    */
    case 109:
    {
      uint8_t extruder;
      /*
      if(setTargetedHotend(109, extruder)){
        break;
      }
      LCD_MESSAGERPGM(_T(MSG_HEATING));
	  heating_status = HeatingStatus::EXTRUDER_HEATING;
	  if (farm_mode) { prusa_statistics(1); };
*/
#ifdef AUTOTEMP
        autotemp_enabled=false;
      #endif
      if (code_seen('S')) {
          code_value();//setTargetHotendSafe(code_value(), extruder);
            } else if (code_seen('R')) {
                code_value();//setTargetHotendSafe(code_value(), extruder);
      }
      #ifdef AUTOTEMP
        if (code_seen('S')) autotemp_min=code_value();
        if (code_seen('B')) autotemp_max=code_value();
        if (code_seen('F'))
        {
          autotemp_factor=code_value();
          autotemp_enabled=true;
        }
      #endif

      codenum = millis();

      /* See if we are heating up or cooling down */
      ;//target_direction = isHeatingHotend(extruder); // true if heating, false if cooling

      cancel_heatup = false;

	  ;//wait_for_heater(codenum, extruder); //loops until target temperature is reached

        /* LCD_MESSAGERPGM(_T(MSG_HEATING_COMPLETE));
		heating_status = HeatingStatus::EXTRUDER_HEATING_COMPLETE;
		if (farm_mode) { prusa_statistics(2); };
        
        //starttime=_millis();
        previous_millis_cmd.start(); */
      }
      break;

    /*!
	### M190 - Wait for bed temperature <a href="https://reprap.org/wiki/G-code#M190:_Wait_for_bed_temperature_to_reach_target_temp">M190: Wait for bed temperature to reach target temp</a>
    #### Usage
    
        M190 [ R | S ]
    
    #### Parameters (not mandatory)
    
	  - `S` - Set extruder temperature and wait for heating
      - `R` - Set extruder temperature and wait for heating or cooling
    
    If no parameter is supplied, waits for heating or cooling to previously set temperature.
	*/
    case 190: 
    {
    #if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
    {
        bool CooldownNoWait = false;
        ;//LCD_MESSAGERPGM(_T(MSG_BED_HEATING));
		heating_status = HeatingStatus::BED_HEATING;
		if (farm_mode) 
    ;//{ prusa_statistics(1); };
        if (code_seen('S')) 
		{
          code_value();//setTargetBed(code_value());
          CooldownNoWait = true;
        } 
		else if (code_seen('R')) 
		{
          code_value();//setTargetBed(code_value());
        }
        codenum = millis();
        
        cancel_heatup = false;
        ;//target_direction = isHeatingBed(); // true if heating, false if cooling
        /* 
        while ( (!cancel_heatup) && (target_direction ? (isHeatingBed()) : (isCoolingBed()&&(CooldownNoWait==false))) )
        {
          if(( millis() - codenum) > 1000 ) //Print Temp Reading every 1 second while heating up.
          {
			  if (!farm_mode) {
				  float tt = degHotend(active_extruder);
				  ;//SERIAL_PROTOCOLPGM("T:");
				  ;//SERIAL_PROTOCOL(tt);
				  ;//SERIAL_PROTOCOLPGM(" E:");
				  ;//SERIAL_PROTOCOL((int)active_extruder);
				  ;//SERIAL_PROTOCOLPGM(" B:");
				  ;//SERIAL_PROTOCOL_F(degBed(), 1);
				  ;//SERIAL_PROTOCOLLN();
			  }
				  codenum = _millis();
			  
          }
          manage_heater();
          manage_inactivity();
          lcd_update(0);
        }
        LCD_MESSAGERPGM(_T(MSG_BED_DONE));
		heating_status = HeatingStatus::BED_HEATING_COMPLETE;

        previous_millis_cmd.start(); */
    }
    #endif
    }
        break;

    #if defined(FAN_PIN) && FAN_PIN > -1

      /*!
	  ### M106 - Set fan speed <a href="https://reprap.org/wiki/G-code#M106:_Fan_On">M106: Fan On</a>
      #### Usage
      
        M106 [ S ]
        
      #### Parameters
      - `S` - Specifies the duty cycle of the print fan. Allowed values are 0-255. If it's omitted, a value of 255 is used.
      */
      case 106: // M106 Sxxx Fan On S<speed> 0 .. 255
        {
          if (code_seen('S')){
           fanSpeed = code_value_uint8();
        }
        else {
          fanSpeed = 255;
        }
        }
        break;

      /*!
	  ### M107 - Fan off <a href="https://reprap.org/wiki/G-code#M107:_Fan_Off">M107: Fan Off</a>
      */
      case 107:
      {
        fanSpeed = 0;
      }
        break;
    #endif //FAN_PIN

    #if defined(PS_ON_PIN) && PS_ON_PIN > -1

      /*!
	  ### M80 - Turn on the Power Supply <a href="https://reprap.org/wiki/G-code#M80:_ATX_Power_On">M80: ATX Power On</a>
      Only works if the firmware is compiled with PS_ON_PIN defined.
      */
      case 80:
        SET_OUTPUT(PS_ON_PIN); //GND
        WRITE(PS_ON_PIN, PS_ON_AWAKE);

        // If you have a switch on suicide pin, this is useful
        // if you want to start another print with suicide feature after
        // a print without suicide...
        #if defined SUICIDE_PIN && SUICIDE_PIN > -1
            SET_OUTPUT(SUICIDE_PIN);
            WRITE(SUICIDE_PIN, HIGH);
        #endif

          powersupply = true;
          LCD_MESSAGERPGM(MSG_WELCOME);
          lcd_update(0);
        break;

      /*!
	  ### M81 - Turn off Power Supply <a href="https://reprap.org/wiki/G-code#M81:_ATX_Power_Off">M81: ATX Power Off</a>
      Only works if the firmware is compiled with PS_ON_PIN defined.
      */
      case 81: 
        disable_heater();
        st_synchronize();
        disable_e0();
        disable_e1();
        disable_e2();
        finishAndDisableSteppers();
        fanSpeed = 0;
        _delay(1000); // Wait a little before to switch off
      #if defined(SUICIDE_PIN) && SUICIDE_PIN > -1
        st_synchronize();
        suicide();
      #elif defined(PS_ON_PIN) && PS_ON_PIN > -1
        SET_OUTPUT(PS_ON_PIN);
        WRITE(PS_ON_PIN, PS_ON_ASLEEP);
      #endif
        powersupply = false;
        LCD_MESSAGERPGM(CAT4(CUSTOM_MENDEL_NAME,PSTR(" "),MSG_OFF,PSTR(".")));
        lcd_update(0);
	  break;
    #endif

    /*!
	### M82 - Set E axis to absolute mode <a href="https://reprap.org/wiki/G-code#M82:_Set_extruder_to_absolute_mode">M82: Set extruder to absolute mode</a>
	Makes the extruder interpret extrusion as absolute positions.
    */
    case 82:
    {
      axis_relative_modes &= ~E_AXIS_MASK;
    }
      break;

    /*!
	### M83 - Set E axis to relative mode <a href="https://reprap.org/wiki/G-code#M83:_Set_extruder_to_relative_mode">M83: Set extruder to relative mode</a>
	Makes the extruder interpret extrusion values as relative positions.
    */
    case 83:
    {
      axis_relative_modes |= E_AXIS_MASK;
    }
      break;

    /*!
	### M84 - Disable steppers <a href="https://reprap.org/wiki/G-code#M84:_Stop_idle_hold">M84: Stop idle hold</a>
    This command can be used to set the stepper inactivity timeout (`S`) or to disable steppers (`X`,`Y`,`Z`,`E`)
	This command can be used without any additional parameters. In that case all steppers are disabled.
    
    The file completeness check uses this parameter to detect an incomplete file. It has to be present at the end of a file with no parameters.
	
        M84 [ S | X | Y | Z | E ]
	
	  - `S` - Seconds
	  - `X` - X axis
	  - `Y` - Y axis
	  - `Z` - Z axis
	  - `E` - Extruder

	### M18 - Disable steppers <a href="https://reprap.org/wiki/G-code#M18:_Disable_all_stepper_motors">M18: Disable all stepper motors</a>
	Equal to M84 (compatibility)
    */
    case 18: //compatibility
    case 84: // M84
    {
      if(code_seen('S')){
        stepper_inactive_time = code_value() * 1000;
      }
      else
      {
        bool all_axis = !((code_seen(axis_codes[X_AXIS])) || (code_seen(axis_codes[Y_AXIS])) || (code_seen(axis_codes[Z_AXIS]))|| (code_seen(axis_codes[E_AXIS])));
        if(all_axis)
        {
          ;//st_synchronize();
          ;//disable_e0();
          ;//disable_e1();
          ;//disable_e2();
          ;//finishAndDisableSteppers();
        }
        else
        {
          ;//st_synchronize();
		  if (code_seen('X')) 
      ;//disable_x();
		  if (code_seen('Y')) 
      ;//disable_y();
		  if (code_seen('Z')) 
      ;//disable_z();
#if ((E0_ENABLE_PIN != X_ENABLE_PIN) && (E1_ENABLE_PIN != Y_ENABLE_PIN)) // Only enable on boards that have seperate ENABLE_PINS
		  if (code_seen('E')) {
			  ;//disable_e0();
			  ;//disable_e1();
			  ;//disable_e2();
            }
          #endif
        }
      }
    }
      break;

    /*!
	### M85 - Set max inactive time <a href="https://reprap.org/wiki/G-code#M85:_Set_Inactivity_Shutdown_Timer">M85: Set Inactivity Shutdown Timer</a>
    #### Usage
    
        M85 [ S ]
    
    #### Parameters
    - `S` - specifies the time in seconds. If a value of 0 is specified, the timer is disabled.
    */
    case 85: // M85
    {
      if(code_seen('S')) {
        max_inactive_time = code_value() * 1000;
      }
    }
    break;
#ifdef SAFETYTIMER

    /*!
    ### M86 - Set safety timer expiration time <a href="https://reprap.org/wiki/G-code#M86:_Set_Safety_Timer_expiration_time">M86: Set Safety Timer expiration time</a>	
    When safety timer expires, heatbed and nozzle target temperatures are set to zero.
    #### Usage
    
        M86 [ S ]
    
    #### Parameters
    - `S` - specifies the time in seconds. If a value of 0 is specified, the timer is disabled.
    */
	case 86: 
  {
	  if (code_seen('S')) {
	    safetytimer_inactive_time = code_value() * 1000;
		;//safetyTimer.start();
	  }
  }
  break;
#endif

    /*!
	### M92 Set Axis steps-per-unit <a href="https://reprap.org/wiki/G-code#M92:_Set_axis_steps_per_unit">M92: Set axis_steps_per_unit</a>
	Allows programming of steps per unit (usually mm) for motor drives. These values are reset to firmware defaults on power on, unless saved to EEPROM if available (M500 in Marlin)
	#### Usage
    
	    M92 [ X | Y | Z | E ]
	
    #### Parameters
	- `X` - Steps per unit for the X drive
	- `Y` - Steps per unit for the Y drive
	- `Z` - Steps per unit for the Z drive
	- `E` - Steps per unit for the extruder drive
    */
    case 92:
    {
      for(int8_t i=0; i < NUM_AXIS; i++)
      {
        if(code_seen(axis_codes[i]))
        {
          if(i == E_AXIS) { // E
            float value = code_value();
            if(value < 20.0) {
              float factor = cs.axis_steps_per_unit[i] / value; // increase e constants if M92 E14 is given for netfab.
              cs.max_jerk[E_AXIS] *= factor;
              max_feedrate[i] *= factor;
              axis_steps_per_sqr_second[i] *= factor;
            }
            cs.axis_steps_per_unit[i] = value;
#if defined(FILAMENT_SENSOR) && defined(PAT9125)
            fsensor_set_axis_steps_per_unit(value);
#endif
          }
          else {
            cs.axis_steps_per_unit[i] = code_value();
          }
        }
      }
      ;//reset_acceleration_rates();
    }
      break;

    /*!
	### M110 - Set Line number <a href="https://reprap.org/wiki/G-code#M110:_Set_Current_Line_Number">M110: Set Current Line Number</a>
	Sets the line number in G-code
	#### Usage
    
	    M110 [ N ]
	
    #### Parameters
	- `N` - Line number
    */
    case 110:
    {
      if (code_seen('N'))
	    gcode_LastN = code_value_long();
    }
    break;

    /*!
    ### M113 - Get or set host keep-alive interval <a href="https://reprap.org/wiki/G-code#M113:_Host_Keepalive">M113: Host Keepalive</a>
    During some lengthy processes, such as G29, Marlin may appear to the host to have “gone away.” The “host keepalive” feature will send messages to the host when Marlin is busy or waiting for user response so the host won’t try to reconnect (or disconnect).
    #### Usage
    
        M113 [ S ]
	
    #### Parameters
	- `S` - Seconds. Default is 2 seconds between "busy" messages
    */
	case 113:
  {
		if (code_seen('S')) {
			;//host_keepalive_interval = code_value_uint8();
//			NOMORE(host_keepalive_interval, 60);
		}
		else {
			;//;//;//SERIAL_ECHO_START;
			;//;//;//SERIAL_ECHOPAIR("M113 S", (unsigned long)host_keepalive_interval);
			;//;//SERIAL_PROTOCOLLN();
		}
  }
		break;

    /*!
	### M115 - Firmware info <a href="https://reprap.org/wiki/G-code#M115:_Get_Firmware_Version_and_Capabilities">M115: Get Firmware Version and Capabilities</a>
    Print the firmware info and capabilities
    Without any arguments, prints Prusa firmware version number, machine type, extruder count and UUID.
    `M115 U` Checks the firmware version provided. If the firmware version provided by the U code is higher than the currently running firmware, it will pause the print for 30s and ask the user to upgrade the firmware.
	
	_Examples:_
	
	`M115` results:
	
	`FIRMWARE_NAME:Prusa-Firmware 3.8.1 based on Marlin FIRMWARE_URL:https://github.com/prusa3d/Prusa-Firmware PROTOCOL_VERSION:1.0 MACHINE_TYPE:Prusa i3 MK3S EXTRUDER_COUNT:1 UUID:00000000-0000-0000-0000-000000000000`
	
	`M115 V` results:
	
	`3.8.1`
	
	`M115 U3.8.2-RC1` results on LCD display for 30s or user interaction:
	
	`New firmware version available: 3.8.2-RC1 Please upgrade.`
    #### Usage
    
        M115 [ V | U ]
	
    #### Parameters
	- V - Report current installed firmware version
	- U - Firmware version provided by G-code to be compared to current one.  
	*/
	case 115: // M115
  {
      if (code_seen('V')) {
          // Report the Prusa version number.
          ;//;//SERIAL_PROTOCOLLNRPGM(FW_VERSION_STR_P());
      } else if (code_seen('U')) {
          // Check the firmware version provided. If the firmware version provided by the U code is higher than the currently running firmware,
          // pause the print for 30s and ask the user to upgrade the firmware.
          show_upgrade_dialog_if_version_newer(++ strchr_pointer);
      } else {
          ;//;//SERIAL_ECHOPGM("FIRMWARE_NAME:Prusa-Firmware ");
          ;//;//SERIAL_ECHORPGM(FW_VERSION_STR_P());
          ;//;//SERIAL_ECHOPGM(" based on Marlin FIRMWARE_URL:https://github.com/prusa3d/Prusa-Firmware PROTOCOL_VERSION:");
          ;//;//SERIAL_ECHOPGM(PROTOCOL_VERSION);
          ;//;//SERIAL_ECHOPGM(" MACHINE_TYPE:");
          ;//;//SERIAL_ECHOPGM(CUSTOM_MENDEL_NAME); 
          ;//;//SERIAL_ECHOPGM(" EXTRUDER_COUNT:"); 
          ;//;//SERIAL_ECHOPGM(STRINGIFY(EXTRUDERS)); 
          ;//;//SERIAL_ECHOPGM(" UUID:"); 
          ;//;//SERIAL_ECHOLNPGM(MACHINE_UUID);
#ifdef EXTENDED_CAPABILITIES_REPORT
          ;//extended_capabilities_report();
#endif //EXTENDED_CAPABILITIES_REPORT
      }
  }
      break;

    /*!
	### M114 - Get current position <a href="https://reprap.org/wiki/G-code#M114:_Get_Current_Position">M114: Get Current Position</a>
    */
    case 114:
		;//gcode_M114();
      break;

      
      /*
        M117 moved up to get the high priority

    case 117: // M117 display message
      starpos = (strchr(strchr_pointer + 5,'*'));
      if(starpos!=NULL)
        *(starpos)='\0';
      lcd_setstatus(strchr_pointer + 5);
      break;*/

#ifdef M120_M121_ENABLED
    /*!
    ### M120 - Enable endstops <a href="https://reprap.org/wiki/G-code#M120:_Enable_endstop_detection">M120: Enable endstop detection</a>
    */
    case 120:
      enable_endstops(true) ;
      break;

    /*!
    ### M121 - Disable endstops <a href="https://reprap.org/wiki/G-code#M121:_Disable_endstop_detection">M121: Disable endstop detection</a>
    */
    case 121:
      enable_endstops(false) ;
      break;
#endif //M120_M121_ENABLED

    /*!
	### M119 - Get endstop states <a href="https://reprap.org/wiki/G-code#M119:_Get_Endstop_Status">M119: Get Endstop Status</a>
	Returns the current state of the configured X, Y, Z endstops. Takes into account any 'inverted endstop' settings, so one can confirm that the machine is interpreting the endstops correctly.
    */
    case 119:
    {
    ;//SERIAL_PROTOCOLRPGM(_N("Reporting endstop status"));////MSG_M119_REPORT
    ;//SERIAL_PROTOCOLLN();
      #if defined(X_MIN_PIN) && X_MIN_PIN > -1
        ;//SERIAL_PROTOCOLRPGM(_n("x_min: "));////MSG_X_MIN
        /*
        if(READ(X_MIN_PIN)^X_MIN_ENDSTOP_INVERTING){
          ;//SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_HIT);
        }else{
          ;//SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_OPEN);
        }
        ;//SERIAL_PROTOCOLLN();
      #endif
      #if defined(X_MAX_PIN) && X_MAX_PIN > -1
        ;//SERIAL_PROTOCOLRPGM(_n("x_max: "));////MSG_X_MAX
        if(READ(X_MAX_PIN)^X_MAX_ENDSTOP_INVERTING){
          ;//SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_HIT);
        }else{
          ;//SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_OPEN);
        }
        ;//SERIAL_PROTOCOLLN();
      #endif
      #if defined(Y_MIN_PIN) && Y_MIN_PIN > -1
        ;//SERIAL_PROTOCOLRPGM(_n("y_min: "));////MSG_Y_MIN
        if(READ(Y_MIN_PIN)^Y_MIN_ENDSTOP_INVERTING){
          ;//SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_HIT);
        }else{
          ;//SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_OPEN);
        }
        ;//SERIAL_PROTOCOLLN();
      #endif
      #if defined(Y_MAX_PIN) && Y_MAX_PIN > -1
        ;//SERIAL_PROTOCOLRPGM(_n("y_max: "));////MSG_Y_MAX
        if(READ(Y_MAX_PIN)^Y_MAX_ENDSTOP_INVERTING){
          ;//SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_HIT);
        }else{
          ;//SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_OPEN);
        }
        ;//SERIAL_PROTOCOLLN();
      #endif
      #if defined(Z_MIN_PIN) && Z_MIN_PIN > -1
        ;//SERIAL_PROTOCOLRPGM(MSG_Z_MIN);
        if(READ(Z_MIN_PIN)^Z_MIN_ENDSTOP_INVERTING){
          ;//SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_HIT);
        }else{
          ;//SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_OPEN);
        }
        ;//SERIAL_PROTOCOLLN();
      #endif
      #if defined(Z_MAX_PIN) && Z_MAX_PIN > -1
        ;//SERIAL_PROTOCOLRPGM(MSG_Z_MAX);
        if(READ(Z_MAX_PIN)^Z_MAX_ENDSTOP_INVERTING){
          ;//SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_HIT);
        }else{
          ;//SERIAL_PROTOCOLRPGM(MSG_ENDSTOP_OPEN);
        }
        ;//SERIAL_PROTOCOLLN();
        */
      #endif
    }
      break;
      //!@todo update for all axes, use for loop

#if (defined(FANCHECK) && (((defined(TACH_0) && (TACH_0 >-1)) || (defined(TACH_1) && (TACH_1 > -1)))))
    /*!
	### M123 - Tachometer value <a href="https://www.reprap.org/wiki/G-code#M123:_Tachometer_value_.28RepRap_.26_Prusa.29">M123: Tachometer value</a>
  This command is used to report fan speeds and fan pwm values.
  #### Usage
    
        M123

    - E0:     - Hotend fan speed in RPM
    - PRN1:   - Part cooling fans speed in RPM
    - E0@:    - Hotend fan PWM value
    - PRN1@:  -Part cooling fan PWM value

  _Example:_

    E0:3240 RPM PRN1:4560 RPM E0@:255 PRN1@:255

    */
    case 123:
    gcode_M123();
    break;
#endif //FANCHECK and TACH_0 and TACH_1

    #ifdef BLINKM
    /*!
	### M150 - Set RGB(W) Color <a href="https://reprap.org/wiki/G-code#M150:_Set_LED_color">M150: Set LED color</a>
	In Prusa Firmware this G-code is deactivated by default, must be turned on in the source code by defining BLINKM and its dependencies.
    #### Usage
    
        M150 [ R | U | B ]
    
    #### Parameters
    - `R` - Red color value
    - `U` - Green color value. It is NOT `G`!
    - `B` - Blue color value
    */
    case 150:
      {
        byte red;
        byte grn;
        byte blu;

        if(code_seen('R')) red = code_value();
        if(code_seen('U')) grn = code_value();
        if(code_seen('B')) blu = code_value();

        SendColors(red,grn,blu);
      }
      break;
    #endif //BLINKM

    /*!
	### M200 - Set filament diameter <a href="https://reprap.org/wiki/G-code#M200:_Set_filament_diameter">M200: Set filament diameter</a>
	#### Usage
    
	    M200 [ D | T ]
	
    #### Parameters
	  - `D` - Diameter in mm
	  - `T` - Number of extruder (MMUs)
    */
    case 200: // M200 D<millimeters> set filament diameter and set E axis units to cubic millimeters (use S0 to set back to millimeters).
      {

        uint8_t extruder = active_extruder;
        if(code_seen('T')) {
          extruder = code_value_uint8();
		  if(extruder >= EXTRUDERS) {
            ;//;//SERIAL_ECHO_START;
            ;//;//SERIAL_ECHO(_n("M200 Invalid extruder "));////MSG_M200_INVALID_EXTRUDER
            break;
          }
        }
        if(code_seen('D')) {
		  float diameter = code_value();
		  if (diameter == 0.0) {
			// setting any extruder filament size disables volumetric on the assumption that
			// slicers either generate in extruder values as cubic mm or as as filament feeds
			// for all extruders
		    cs.volumetric_enabled = false;
		  } else {
            cs.filament_size[extruder] = code_value();
			// make sure all extruders have some sane value for the filament size
			cs.filament_size[0] = (cs.filament_size[0] == 0.0 ? DEFAULT_NOMINAL_FILAMENT_DIA : cs.filament_size[0]);
            #if EXTRUDERS > 1
				cs.filament_size[1] = (cs.filament_size[1] == 0.0 ? DEFAULT_NOMINAL_FILAMENT_DIA : cs.filament_size[1]);
				#if EXTRUDERS > 2
					cs.filament_size[2] = (cs.filament_size[2] == 0.0 ? DEFAULT_NOMINAL_FILAMENT_DIA : cs.filament_size[2]);
				#endif
            #endif
			cs.volumetric_enabled = true;
		  }
        } else {
          //reserved for setting filament diameter via UFID or filament measuring device
          break;
        }
		;//calculate_extruder_multipliers();
      }
      break;

    /*!
	### M201 - Set Print Max Acceleration <a href="https://reprap.org/wiki/G-code#M201:_Set_max_printing_acceleration">M201: Set max printing acceleration</a>
    For each axis individually.
    */
    case 201:
    {
		for (int8_t i = 0; i < NUM_AXIS; i++)
		{
			if (code_seen(axis_codes[i]))
			{
				unsigned long val = code_value();
#ifdef TMC2130
				unsigned long val_silent = val;
				if ((i == X_AXIS) || (i == Y_AXIS))
				{
					if (val > NORMAL_MAX_ACCEL_XY)
						val = NORMAL_MAX_ACCEL_XY;
					if (val_silent > SILENT_MAX_ACCEL_XY)
						val_silent = SILENT_MAX_ACCEL_XY;
				}
				cs.max_acceleration_units_per_sq_second_normal[i] = val;
				cs.max_acceleration_units_per_sq_second_silent[i] = val_silent;
#else //TMC2130
				max_acceleration_units_per_sq_second[i] = val;
#endif //TMC2130
			}
		}
		// steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
		;//reset_acceleration_rates();
    }
		break;
    #if 0 // Not used for Sprinter/grbl gen6
    case 202: // M202
      for(int8_t i=0; i < NUM_AXIS; i++) {
        if(code_seen(axis_codes[i])) axis_travel_steps_per_sqr_second[i] = code_value() * cs.axis_steps_per_unit[i];
      }
      break;
    #endif

    /*!
	### M203 - Set Max Feedrate <a href="https://reprap.org/wiki/G-code#M203:_Set_maximum_feedrate">M203: Set maximum feedrate</a>
    For each axis individually.
    */
    case 203: // M203 max feedrate mm/sec
    {
		for (uint8_t i = 0; i < NUM_AXIS; i++)
		{
			if (code_seen(axis_codes[i]))
			{
				float val = code_value();
#ifdef TMC2130
				float val_silent = val;
				if ((i == X_AXIS) || (i == Y_AXIS))
				{
					if (val > NORMAL_MAX_FEEDRATE_XY)
						val = NORMAL_MAX_FEEDRATE_XY;
					if (val_silent > SILENT_MAX_FEEDRATE_XY)
						val_silent = SILENT_MAX_FEEDRATE_XY;
				}
				cs.max_feedrate_normal[i] = val;
				cs.max_feedrate_silent[i] = val_silent;
#else //TMC2130
				max_feedrate[i] = val;
#endif //TMC2130
			}
		}
    }
		break;

    /*!
	### M204 - Acceleration settings <a href="https://reprap.org/wiki/G-code#M204:_Set_default_acceleration">M204: Set default acceleration</a>

    #### Old format:
    ##### Usage
    
        M204 [ S | T ]
        
    ##### Parameters
    - `S` - normal moves
    - `T` - filmanent only moves
    
    #### New format:
    ##### Usage
    
        M204 [ P | R | T ]
    
    ##### Parameters
    - `P` - printing moves
    - `R` - filmanent only moves
    - `T` - travel moves (as of now T is ignored)
	*/
    case 204:
      {
        if(code_seen('S')) {
          // Legacy acceleration format. This format is used by the legacy Marlin, MK2 or MK3 firmware,
          // and it is also generated by Slic3r to control acceleration per extrusion type
          // (there is a separate acceleration settings in Slicer for perimeter, first layer etc).
          cs.acceleration = cs.travel_acceleration = code_value();
          // Interpret the T value as retract acceleration in the old Marlin format.
          if(code_seen('T'))
            cs.retract_acceleration = code_value();
        } else {
          // New acceleration format, compatible with the upstream Marlin.
          if(code_seen('P'))
            cs.acceleration = code_value();
          if(code_seen('R'))
            cs.retract_acceleration = code_value();
          if(code_seen('T'))
            cs.travel_acceleration = code_value();
        }
      }
      break;

    /*!
	### M205 - Set advanced settings <a href="https://reprap.org/wiki/G-code#M205:_Advanced_settings">M205: Advanced settings</a>
    Set some advanced settings related to movement.
    #### Usage
    
        M205 [ S | T | B | X | Y | Z | E ]
        
    #### Parameters
    - `S` - Minimum feedrate for print moves (unit/s)
    - `T` - Minimum feedrate for travel moves (units/s)
    - `B` - Minimum segment time (us)
    - `X` - Maximum X jerk (units/s)
    - `Y` - Maximum Y jerk (units/s)
    - `Z` - Maximum Z jerk (units/s)
    - `E` - Maximum E jerk (units/s)
    */
    case 205: 
    {
      if(code_seen('S')) cs.minimumfeedrate = code_value();
      if(code_seen('T')) cs.mintravelfeedrate = code_value();
      if(code_seen('B')) cs.minsegmenttime = code_value() ;
      if(code_seen('X')) cs.max_jerk[X_AXIS] = cs.max_jerk[Y_AXIS] = code_value();
      if(code_seen('Y')) cs.max_jerk[Y_AXIS] = code_value();
      if(code_seen('Z')) cs.max_jerk[Z_AXIS] = code_value();
      if(code_seen('E'))
      {
          float e = code_value();
#ifndef LA_NOCOMPAT
          ;//e = la10c_jerk(e);
#endif
          cs.max_jerk[E_AXIS] = e;
      }
    }
    break;

    /*!
	### M206 - Set additional homing offsets <a href="https://reprap.org/wiki/G-code#M206:_Offset_axes">M206: Offset axes</a>
    #### Usage
    
        M206 [ X | Y | Z ]
    
    #### Parameters
    - `X` - X axis offset
    - `Y` - Y axis offset
    - `Z` - Z axis offset
	*/
    case 206:
    {
      for(uint8_t i=0; i < 3; i++)
      {
        if(code_seen(axis_codes[i])) cs.add_homing[i] = code_value();
      }
    }  
    break;


#ifdef FWRETRACT
    /*!
	### M207 - Set firmware retraction <a href="https://reprap.org/wiki/G-code#M207:_Set_retract_length">M207: Set retract length</a>
	#### Usage
    
        M207 [ S | F | Z ]
    
    #### Parameters
    - `S` - positive length to retract, in mm
    - `F` - retraction feedrate, in mm/min
    - `Z` - additional zlift/hop
    */
    case 207: //M207 - set retract length S[positive mm] F[feedrate mm/min] Z[additional zlift/hop]
    {
      if(code_seen('S'))
      {
        cs.retract_length = code_value() ;
      }
      if(code_seen('F'))
      {
        cs.retract_feedrate = code_value()/60 ;
      }
      if(code_seen('Z'))
      {
        cs.retract_zlift = code_value() ;
      }
    }break;

    /*!
	### M208 - Set retract recover length <a href="https://reprap.org/wiki/G-code#M208:_Set_unretract_length">M208: Set unretract length</a>
	#### Usage
    
        M208 [ S | F ]
    
    #### Parameters
    - `S` - positive length surplus to the M207 Snnn, in mm
    - `F` - feedrate, in mm/sec
    */
    case 208: // M208 - set retract recover length S[positive mm surplus to the M207 S*] F[feedrate mm/min]
    {
      if(code_seen('S'))
      {
        cs.retract_recover_length = code_value() ;
      }
      if(code_seen('F'))
      {
        cs.retract_recover_feedrate = code_value()/60 ;
      }
    }break;

    /*!
	### M209 - Enable/disable automatict retract <a href="https://reprap.org/wiki/G-code#M209:_Enable_automatic_retract">M209: Enable automatic retract</a>
	This boolean value S 1=true or 0=false enables automatic retract detect if the slicer did not support G10/G11: every normal extrude-only move will be classified as retract depending on the direction.
    #### Usage
    
        M209 [ S ]
        
    #### Parameters
    - `S` - 1=true or 0=false
    */
    case 209: // M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
    {
      if(code_seen('S'))
      {
        switch(code_value_uint8())
        {
          case 0: 
          {
            cs.autoretract_enabled=false;
            retracted[0]=false;
            #if EXTRUDERS > 1
              retracted[1]=false;
            #endif
            #if EXTRUDERS > 2
              retracted[2]=false;
            #endif
          }break;
          case 1: 
          {
            cs.autoretract_enabled=true;
            retracted[0]=false;
            #if EXTRUDERS > 1
              retracted[1]=false;
            #endif
            #if EXTRUDERS > 2
              retracted[2]=false;
            #endif
          }break;
          default:
            ;//;//SERIAL_ECHO_START;
            ;//;//SERIAL_ECHORPGM(MSG_UNKNOWN_COMMAND);
            ;//;//SERIAL_ECHO(CMDBUFFER_CURRENT_STRING);
            ;//;//SERIAL_ECHOLNPGM("\"(1)");
        }
      }

    }break;
    #endif // FWRETRACT
    /*!
    ### M214 - Set Arc configuration values (Use M500 to store in eeprom)

    #### Usage

        M214 [P] [S] [N] [R] [F]

    #### Parameters
    - `P` - A float representing the max and default millimeters per arc segment.  Must be greater than 0.
    - `S` - A float representing the minimum allowable millimeters per arc segment.  Set to 0 to disable
    - `N` - An int representing the number of arcs to draw before correcting the small angle approximation.  Set to 0 to disable.
    - `R` - An int representing the minimum number of segments per arcs of any radius,
            except when the results in segment lengths greater than or less than the minimum
            and maximum segment length.  Set to 0 to disable.
    - 'F' - An int representing the number of segments per second, unless this results in segment lengths
            greater than or less than the minimum and maximum segment length.  Set to 0 to disable.
    */
    case 214: //!@n M214 - Set Arc Parameters (Use M500 to store in eeprom) P<MM_PER_ARC_SEGMENT> S<MIN_MM_PER_ARC_SEGMENT> R<MIN_ARC_SEGMENTS> F<ARC_SEGMENTS_PER_SEC>
    {
        // Extract all possible parameters if they appear
        float p = code_seen('P') ? code_value_float() : cs.mm_per_arc_segment;
        float s = code_seen('S') ? code_value_float() : cs.min_mm_per_arc_segment;
        unsigned char n = code_seen('N') ? code_value() : cs.n_arc_correction;
        unsigned short r = code_seen('R') ? code_value() : cs.min_arc_segments;
        unsigned short f = code_seen('F') ? code_value() : cs.arc_segments_per_sec;

        // Ensure mm_per_arc_segment is greater than 0, and that min_mm_per_arc_segment is sero or greater than or equal to mm_per_arc_segment
        if (p <=0 || s < 0 || p < s)
        {
            // Should we display some error here?
            break;
        }

        cs.mm_per_arc_segment = p;
        cs.min_mm_per_arc_segment = s;
        cs.n_arc_correction = n;
        cs.min_arc_segments = r;
        cs.arc_segments_per_sec = f;
    }break;
    #if EXTRUDERS > 1

    /*!
	### M218 - Set hotend offset <a href="https://reprap.org/wiki/G-code#M218:_Set_Hotend_Offset">M218: Set Hotend Offset</a>
	In Prusa Firmware this G-code is only active if `EXTRUDERS` is higher then 1 in the source code. On Original i3 Prusa MK2/s MK2.5/s MK3/s it is not active.
    #### Usage
    
        M218 [ X | Y ]
        
    #### Parameters
    - `X` - X offset
    - `Y` - Y offset
    */
    case 218: // M218 - set hotend offset (in mm), T<extruder_number> X<offset_on_X> Y<offset_on_Y>
    {
      uint8_t extruder;
      if(setTargetedHotend(218, extruder)){
        break;
      }
      if(code_seen('X'))
      {
        extruder_offset[X_AXIS][extruder] = code_value();
      }
      if(code_seen('Y'))
      {
        extruder_offset[Y_AXIS][extruder] = code_value();
      }
      ;//;//SERIAL_ECHO_START;
      ;//;//SERIAL_ECHORPGM(MSG_HOTEND_OFFSET);
      for(extruder = 0; extruder < EXTRUDERS; extruder++)
      {
         ;//;//SERIAL_ECHO(" ");
         ;//;//SERIAL_ECHO(extruder_offset[X_AXIS][extruder]);
         ;//;//SERIAL_ECHO(",");
         ;//;//SERIAL_ECHO(extruder_offset[Y_AXIS][extruder]);
      }
      ;//;//SERIAL_ECHOLN("");
    }break;
    #endif

    /*!
	### M220 Set feedrate percentage <a href="https://reprap.org/wiki/G-code#M220:_Set_speed_factor_override_percentage">M220: Set speed factor override percentage</a>
	#### Usage
    
        M220 [ B | S | R ]
    
    #### Parameters
    - `B` - Backup current speed factor
	- `S` - Speed factor override percentage (0..100 or higher)
	- `R` - Restore previous speed factor
    */
    case 220: // M220 S<factor in percent>- set speed factor override percentage
    {
        bool codesWereSeen = false;
        if (code_seen('B')) //backup current speed factor
        {
            saved_feedmultiply_mm = feedmultiply;
            codesWereSeen = true;
        }
        if (code_seen('S'))
        {
            feedmultiply = code_value_short();
            codesWereSeen = true;
        }
        if (code_seen('R')) //restore previous feedmultiply
        {
            feedmultiply = saved_feedmultiply_mm;
            codesWereSeen = true;
        }
        if (!codesWereSeen)
        {
            ;//printf_P(PSTR("%i%%\n"), feedmultiply);
        }
    }
    break;

    /*!
	### M221 - Set extrude factor override percentage <a href="https://reprap.org/wiki/G-code#M221:_Set_extrude_factor_override_percentage">M221: Set extrude factor override percentage</a>
	#### Usage
    
        M221 [ S | T ]
    
    #### Parameters
	- `S` - Extrude factor override percentage (0..100 or higher), default 100%
	- `T` - Extruder drive number (Prusa Firmware only), default 0 if not set.
    */
    case 221: // M221 S<factor in percent>- set extrude factor override percentage
    {
        if (code_seen('S'))
        {
            int tmp_code = code_value_short();
            if (code_seen('T'))
            {
                uint8_t extruder;
                /*
                if (setTargetedHotend(221, extruder))
                    break;
                extruder_multiply[extruder] = tmp_code;*/
            }
            else
            {
                extrudemultiply = tmp_code ;
            }
        }
        else
        {
            ;//printf_P(PSTR("%i%%\n"), extrudemultiply);
        }
        ;//calculate_extruder_multipliers();
    }
    break;

    /*!
    ### M226 - Wait for Pin state <a href="https://reprap.org/wiki/G-code#M226:_Wait_for_pin_state">M226: Wait for pin state</a>
    Wait until the specified pin reaches the state required
    #### Usage
    
        M226 [ P | S ]
    
    #### Parameters
    - `P` - pin number
    - `S` - pin state
    */
	case 226: // M226 P<pin number> S<pin state>- Wait until the specified pin reaches the state required
	{
      if(code_seen('P')){
        int pin_number = code_value_short(); // pin number
        int pin_state = -1; // required pin state - default is inverted

        if(code_seen('S')) pin_state = code_value_short(); // required pin state

        if(pin_state >= -1 && pin_state <= 1){

          for(int8_t i = 0; i < (int8_t)(sizeof(sensitive_pins)/sizeof(sensitive_pins[0])); i++)
          {
            if (((int8_t)sensitive_pins[i] == pin_number))
            {
              pin_number = -1;
              break;
            }
          }

          /* if (pin_number > -1)
          {
            int target = LOW;

            st_synchronize();

            pinMode(pin_number, INPUT);

            switch(pin_state){
            case 1:
              target = HIGH;
              break;

            case 0:
              target = LOW;
              break;

            case -1:
              target = !digitalRead(pin_number);
              break;
            }

            while(digitalRead(pin_number) != target){
              manage_heater();
              manage_inactivity();
              lcd_update(0);
            }
          } */
        }
      }
    }
    break;

    #if NUM_SERVOS > 0

    /*!
	### M280 - Set/Get servo position <a href="https://reprap.org/wiki/G-code#M280:_Set_servo_position">M280: Set servo position</a>
	In Prusa Firmware this G-code is deactivated by default, must be turned on in the source code.
    #### Usage
    
        M280 [ P | S ]
    
    #### Parameters
    - `P` - Servo index (id)
    - `S` - Target position
    */
    case 280: // M280 - set servo position absolute. P: servo index, S: angle or microseconds
      {
        int servo_index = -1;
        int servo_position = 0;
        if (code_seen('P'))
          servo_index = code_value();
        if (code_seen('S')) {
          servo_position = code_value();
          if ((servo_index >= 0) && (servo_index < NUM_SERVOS)) {
#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
		      servos[servo_index].attach(0);
#endif
            servos[servo_index].write(servo_position);
#if defined (ENABLE_AUTO_BED_LEVELING) && (PROBE_SERVO_DEACTIVATION_DELAY > 0)
              _delay(PROBE_SERVO_DEACTIVATION_DELAY);
              servos[servo_index].detach();
#endif
          }
          else {
            ;//;//SERIAL_ECHO_START;
            ;//;//SERIAL_ECHO("Servo ");
            ;//;//SERIAL_ECHO(servo_index);
            ;//;//SERIAL_ECHOLN(" out of range");
          }
        }
        else if (servo_index >= 0) {
          ;//SERIAL_PROTOCOL(MSG_OK);
          ;//SERIAL_PROTOCOL(" Servo ");
          ;//SERIAL_PROTOCOL(servo_index);
          ;//SERIAL_PROTOCOL(": ");
          ;//SERIAL_PROTOCOLLN(servos[servo_index].read());
        }
      }
      break;
    #endif // NUM_SERVOS > 0

    #if (LARGE_FLASH == true && ( BEEPER > 0 || defined(ULTRALCD) || defined(LCD_USE_I2C_BUZZER)))
    
    /*!
	### M300 - Play tone <a href="https://reprap.org/wiki/G-code#M300:_Play_beep_sound">M300: Play beep sound</a>
	In Prusa Firmware the defaults are `100Hz` and `1000ms`, so that `M300` without parameters will beep for a second.
    #### Usage
    
        M300 [ S | P ]
    
    #### Parameters
    - `S` - frequency in Hz. Not all firmware versions support this parameter
    - `P` - duration in milliseconds
    */
    case 300: // M300
    {
      uint16_t beepS = code_seen('S') ? code_value() : 110;
      uint16_t beepP = code_seen('P') ? code_value() : 1000;
      if (beepS > 0)
      {
        #if BEEPER > 0
          Sound_MakeCustom(beepP,beepS,false);
        #endif
      }
      else
      {
        _delay(beepP);
      }
    }
    break;
    #endif // M300

    #ifdef PIDTEMP

    /*!
	### M301 - Set hotend PID <a href="https://reprap.org/wiki/G-code#M301:_Set_PID_parameters">M301: Set PID parameters</a>
	Sets Proportional (P), Integral (I) and Derivative (D) values for hot end.
    See also <a href="https://reprap.org/wiki/PID_Tuning">PID Tuning.</a>
    #### Usage
    
        M301 [ P | I | D ]
    
    #### Parameters
    - `P` - proportional (Kp)
    - `I` - integral (Ki)
    - `D` - derivative (Kd)
    */
    case 301:
      {
        if(code_seen('P')) cs.Kp = code_value();
        if(code_seen('I')) cs.Ki = code_value();//scalePID_i(code_value());
        if(code_seen('D')) cs.Kd = code_value();//scalePID_d(code_value());

        ;//updatePID();
        ;//SERIAL_PROTOCOLRPGM(MSG_OK);
        ;//SERIAL_PROTOCOLPGM(" p:");
        ;//SERIAL_PROTOCOL(cs.Kp);
        ;//SERIAL_PROTOCOLPGM(" i:");
        ;//SERIAL_PROTOCOL(unscalePID_i(cs.Ki));
        ;//SERIAL_PROTOCOLPGM(" d:");
        ;//SERIAL_PROTOCOL(unscalePID_d(cs.Kd));
        ;//SERIAL_PROTOCOLLN();
      }
      break;
    #endif //PIDTEMP
    #ifdef PIDTEMPBED

    /*!
	### M304 - Set bed PID  <a href="https://reprap.org/wiki/G-code#M304:_Set_PID_parameters_-_Bed">M304: Set PID parameters - Bed</a>
	Sets Proportional (P), Integral (I) and Derivative (D) values for bed.
    See also <a href="https://reprap.org/wiki/PID_Tuning">PID Tuning.</a>
    #### Usage
    
        M304 [ P | I | D ]
    
    #### Parameters
    - `P` - proportional (Kp)
    - `I` - integral (Ki)
    - `D` - derivative (Kd)
    */
    case 304:
      {
        if(code_seen('P')) cs.bedKp = code_value();
        if(code_seen('I')) cs.bedKi = code_value();//scalePID_i(code_value());
        if(code_seen('D')) cs.bedKd = code_value();//scalePID_d(code_value());

        ;//updatePID();
       	;//SERIAL_PROTOCOLRPGM(MSG_OK);
        ;//SERIAL_PROTOCOLPGM(" p:");
        ;//SERIAL_PROTOCOL(cs.bedKp);
        ;//SERIAL_PROTOCOLPGM(" i:");
        ;//SERIAL_PROTOCOL(unscalePID_i(cs.bedKi));
        ;//SERIAL_PROTOCOLPGM(" d:");
        ;//SERIAL_PROTOCOLLN(unscalePID_d(cs.bedKd));
      }
      break;
    #endif //PIDTEMP

    /*!
	### M240 - Trigger camera <a href="https://reprap.org/wiki/G-code#M240:_Trigger_camera">M240: Trigger camera</a>
	
	In Prusa Firmware this G-code is deactivated by default, must be turned on in the source code.
	
	You need to (re)define and assign `CHDK` or `PHOTOGRAPH_PIN` the correct pin number to be able to use the feature.
    */
    case 240: // M240  Triggers a camera by emulating a Canon RC-1 : http://www.doc-diy.net/photo/rc-1_hacked/
     {
     	#ifdef CHDK
       
         SET_OUTPUT(CHDK);
         WRITE(CHDK, HIGH);
         chdkHigh = _millis();
         chdkActive = true;
       
       #else
     	
      	#if defined(PHOTOGRAPH_PIN) && PHOTOGRAPH_PIN > -1
	const uint8_t NUM_PULSES=16;
	const float PULSE_LENGTH=0.01524;
	for(int i=0; i < NUM_PULSES; i++) {
        WRITE(PHOTOGRAPH_PIN, HIGH);
        _delay_ms(PULSE_LENGTH);
        WRITE(PHOTOGRAPH_PIN, LOW);
        _delay_ms(PULSE_LENGTH);
        }
        _delay(7.33);
        for(int i=0; i < NUM_PULSES; i++) {
        WRITE(PHOTOGRAPH_PIN, HIGH);
        _delay_ms(PULSE_LENGTH);
        WRITE(PHOTOGRAPH_PIN, LOW);
        _delay_ms(PULSE_LENGTH);
        }
      	#endif
      #endif //chdk end if
     }
    break;
    #ifdef PREVENT_DANGEROUS_EXTRUDE

    /*!
	### M302 - Allow cold extrude, or set minimum extrude temperature <a href="https://reprap.org/wiki/G-code#M302:_Allow_cold_extrudes">M302: Allow cold extrudes</a>
    This tells the printer to allow movement of the extruder motor above a certain temperature, or if disabled, to allow extruder movement when the hotend is below a safe printing temperature.
    #### Usage
    
        M302 [ S ]
    
    #### Parameters
    - `S` - Cold extrude minimum temperature
    */
    case 302:
    {
	  float temp = .0;
	  if (code_seen('S')) temp=code_value();
      ;//set_extrude_min_temp(temp);
    }
    break;
	#endif

    /*!
	### M303 - PID autotune <a href="https://reprap.org/wiki/G-code#M303:_Run_PID_tuning">M303: Run PID tuning</a>
    PID Tuning refers to a control algorithm used in some repraps to tune heating behavior for hot ends and heated beds. This command generates Proportional (Kp), Integral (Ki), and Derivative (Kd) values for the hotend or bed. Send the appropriate code and wait for the output to update the firmware values.
    #### Usage
    
        M303 [ E | S | C ]
    
    #### Parameters
      - `E` - Extruder, default `E0`. Use `E-1` to calibrate the bed PID
      - `S` - Target temperature, default `210°C` for hotend, 70 for bed
      - `C` - Cycles, default `5`
	*/
    case 303:
    {
      float temp = 150.0;
      int e = 0;
      int c = 5;
      if (code_seen('E')) e = code_value_short();
        if (e < 0)
          temp = 70;
      if (code_seen('S')) temp = code_value();
      if (code_seen('C')) c = code_value_short();
      ;//PID_autotune(temp, e, c);
    }
    break;
    
    /*!
	### M400 - Wait for all moves to finish <a href="https://reprap.org/wiki/G-code#M400:_Wait_for_current_moves_to_finish">M400: Wait for current moves to finish</a>
	Finishes all current moves and and thus clears the buffer.
    Equivalent to `G4` with no parameters.
    */
    case 400:
    {
      ;//st_synchronize();
    }
    break;

    /*!
	### M403 - Set filament type (material) for particular extruder and notify the MMU <a href="https://reprap.org/wiki/G-code#M403:_Set_filament_type_.28material.29_for_particular_extruder_and_notify_the_MMU.">M403 - Set filament type (material) for particular extruder and notify the MMU</a>
    Currently three different materials are needed (default, flex and PVA).  
    And storing this information for different load/unload profiles etc. in the future firmware does not have to wait for "ok" from MMU.
    #### Usage
    
        M403 [ E | F ]
    
    #### Parameters
    - `E` - Extruder number. 0-indexed.
    - `F` - Filament type
	*/
    case 403:
	{
		// currently three different materials are needed (default, flex and PVA)
		// add storing this information for different load/unload profiles etc. in the future
		// firmware does not wait for "ok" from mmu
		if (true/*mmu_enabled*/)
		{
			uint8_t extruder = 255;
			uint8_t filament = FILAMENT_UNDEFINED;
			if(code_seen('E')) extruder = code_value_uint8();
			if(code_seen('F')) filament = code_value_uint8();
			;//mmu_set_filament_type(extruder, filament);
		}
	}
	break;

    /*!
	### M500 - Store settings in EEPROM <a href="https://reprap.org/wiki/G-code#M500:_Store_parameters_in_non-volatile_storage">M500: Store parameters in non-volatile storage</a>
	Save current parameters to EEPROM.
    */
    case 500:
    {
        ;//Config_StoreSettings();
    }
    break;

    /*!
	### M501 - Read settings from EEPROM <a href="https://reprap.org/wiki/G-code#M501:_Read_parameters_from_EEPROM">M501: Read parameters from EEPROM</a>
	Set the active parameters to those stored in the EEPROM. This is useful to revert parameters after experimenting with them.
    */
    case 501:
    {
        ;//Config_RetrieveSettings();
    }
    break;

    /*!
	### M502 - Revert all settings to factory default <a href="https://reprap.org/wiki/G-code#M502:_Restore_Default_Settings">M502: Restore Default Settings</a>
	This command resets all tunable parameters to their default values, as set in the firmware's configuration files. This doesn't reset any parameters stored in the EEPROM, so it must be followed by M500 to write the default settings.
    */
    case 502:
    {
        ;//Config_ResetDefault();
    }
    break;

    /*!
	### M503 - Repport all settings currently in memory <a href="https://reprap.org/wiki/G-code#M503:_Report_Current_Settings">M503: Report Current Settings</a>
	This command asks the firmware to reply with the current print settings as set in memory. Settings will differ from EEPROM contents if changed since the last load / save. The reply output includes the G-Code commands to produce each setting. For example, Steps-Per-Unit values are displayed as an M92 command.
    */
    case 503:
    {
        ;//Config_PrintSettings();
    }
    break;

    /*!
	### M509 - Force language selection <a href="https://reprap.org/wiki/G-code#M509:_Force_language_selection">M509: Force language selection</a>
	Resets the language to English.
	Only on Original Prusa i3 MK2.5/s and MK3/s with multiple languages.
	*/
    case 509:
    {
		;//lang_reset();
        ;//;//SERIAL_ECHO_START;
        ;//SERIAL_PROTOCOLPGM(("LANG SEL FORCED"));
    }
    break;
    #ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED

    /*!
	### M540 - Abort print on endstop hit (enable/disable) <a href="https://reprap.org/wiki/G-code#M540_in_Marlin:_Enable.2FDisable_.22Stop_SD_Print_on_Endstop_Hit.22">M540 in Marlin: Enable/Disable "Stop SD Print on Endstop Hit"</a>
	In Prusa Firmware this G-code is deactivated by default, must be turned on in the source code. You must define `ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED`.
    #### Usage
    
        M540 [ S ]
    
    #### Parameters
    - `S` - disabled=0, enabled=1
	*/
    case 540:
    {
        if(code_seen('S')) abort_on_endstop_hit = code_value() > 0;
    }
    break;
    #endif

	/*!
	### M851 - Set Z-Probe Offset <a href="https://reprap.org/wiki/G-code#M851:_Set_Z-Probe_Offset">M851: Set Z-Probe Offset"</a>
    Sets the Z-probe Z offset. This offset is used to determine the actual Z position of the nozzle when using a probe to home Z with G28. This value may also be used by G81 (Prusa) / G29 (Marlin) to apply correction to the Z position.
	This value represents the distance from nozzle to the bed surface at the point where the probe is triggered. This value will be negative for typical switch probes, inductive probes, and setups where the nozzle makes a circuit with a raised metal contact. This setting will be greater than zero on machines where the nozzle itself is used as the probe, pressing down on the bed to press a switch. (This is a common setup on delta machines.)
    #### Usage
    
        M851 [ Z ]
    
    #### Parameters
    - `Z` - Z offset probe to nozzle.
	*/
    #ifdef CUSTOM_M_CODE_SET_Z_PROBE_OFFSET
    case CUSTOM_M_CODE_SET_Z_PROBE_OFFSET:
    {
      float value;
      if (code_seen('Z'))
      {
        value = code_value();
        if ((Z_PROBE_OFFSET_RANGE_MIN <= value) && (value <= Z_PROBE_OFFSET_RANGE_MAX))
        {
          cs.zprobe_zoffset = -value; // compare w/ line 278 of ConfigurationStore.cpp
          ;//;//SERIAL_ECHO_START;
          ;//;//SERIAL_ECHOLNRPGM(CAT4(MSG_ZPROBE_ZOFFSET, " ", MSG_OK,PSTR("")));
          ;//SERIAL_PROTOCOLLN();
        }
        else
        {
          ;//;//SERIAL_ECHO_START;
          ;//;//SERIAL_ECHORPGM(MSG_ZPROBE_ZOFFSET);
          ;//;//SERIAL_ECHORPGM(MSG_Z_MIN);
          ;//;//SERIAL_ECHO(Z_PROBE_OFFSET_RANGE_MIN);
          ;//;//SERIAL_ECHORPGM(MSG_Z_MAX);
          ;//;//SERIAL_ECHO(Z_PROBE_OFFSET_RANGE_MAX);
          ;//SERIAL_PROTOCOLLN();
        }
      }
      else
      {
          ;//;//SERIAL_ECHO_START;
          ;//;//SERIAL_ECHOLNRPGM(CAT2(MSG_ZPROBE_ZOFFSET, PSTR(" : ")));
          ;//;//SERIAL_ECHO(-cs.zprobe_zoffset);
          ;//SERIAL_PROTOCOLLN();
      }
    }
    break;
    #endif // CUSTOM_M_CODE_SET_Z_PROBE_OFFSET

	/*!
	### M552 - Set IP address <a href="https://reprap.org/wiki/G-code#M552:_Set_IP_address.2C_enable.2Fdisable_network_interface">M552: Set IP address, enable/disable network interface"</a>
    Sets the printer IP address that is shown in the support menu. Designed to be used with the help of host software.
    If P is not specified nothing happens.
    If the structure of the IP address is invalid, 0.0.0.0 is assumed and nothing is shown on the screen in the Support menu.
    #### Usage
    
        M552 [ P<IP_address> ]
    
    #### Parameters
    - `P` - The IP address in xxx.xxx.xxx.xxx format. Eg: P192.168.1.14
	*/
    case 552:
    {
        if (code_seen('P'))
        {
            uint8_t valCnt = 0;
            IP_address = 0;
            do
            {
                *strchr_pointer = '*';
                ((uint8_t*)&IP_address)[valCnt] = code_value_short();
                valCnt++;
            } while ((valCnt < 4) && code_seen('.'));
            
            if (valCnt != 4)
                IP_address = 0;
        }
    } break;

    #ifdef FILAMENTCHANGEENABLE

    /*!
	### M600 - Initiate Filament change procedure <a href="https://reprap.org/wiki/G-code#M600:_Filament_change_pause">M600: Filament change pause</a>
    Initiates Filament change, it is also used during Filament Runout Sensor process.
	If the `M600` is triggered under 25mm it will do a Z-lift of 25mm to prevent a filament blob.
    #### Usage
    
        M600 [ X | Y | Z | E | L | AUTO ]
      
    - `X`    - X position, default 211
    - `Y`    - Y position, default 0
    - `Z`    - relative lift Z, default 2.
    - `E`    - initial retract, default -2
    - `L`    - later retract distance for removal, default -80
    - `AUTO` - Automatically (only with MMU)
    */
    case 600: //Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
	{
		;//st_synchronize();

		float x_position = current_position[X_AXIS];
		float y_position = current_position[Y_AXIS];
		float z_shift = 0; // is it necessary to be a float?
		float e_shift_init = 0;
		float e_shift_late = 0;
		bool automatic = false;
		
        //Retract extruder
        if(code_seen('E'))
        {
          e_shift_init = code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_FIRSTRETRACT
            e_shift_init = FILAMENTCHANGE_FIRSTRETRACT ;
          #endif
        }

		//currently don't work as we are using the same unload sequence as in M702, needs re-work 
		if (code_seen('L'))
		{
			e_shift_late = code_value();
		}
		else
		{
		  #ifdef FILAMENTCHANGE_FINALRETRACT
			e_shift_late = FILAMENTCHANGE_FINALRETRACT;
		  #endif	
		}

        //Lift Z
        if(code_seen('Z'))
        {
          z_shift = code_value();
        }
        else
        {
			z_shift = 0;//gcode_M600_filament_change_z_shift<uint8_t>();
        }
		//Move XY to side
        if(code_seen('X'))
        {
          x_position = code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_XPOS
			x_position = FILAMENTCHANGE_XPOS;
          #endif
        }
        if(code_seen('Y'))
        {
          y_position = code_value();
        }
        else
        {
          #ifdef FILAMENTCHANGE_YPOS
            y_position = FILAMENTCHANGE_YPOS ;
          #endif
        }

		if (true/*mmu_enabled*/ && code_seen_P(PSTR("AUTO")))
			automatic = true;

		;//gcode_M600(automatic, x_position, y_position, z_shift, e_shift_init, e_shift_late);
	
	}
    break;
    #endif //FILAMENTCHANGEENABLE

    /*!
    ### M601 - Pause print <a href="https://reprap.org/wiki/G-code#M601:_Pause_print">M601: Pause print</a>
    */
    /*!
    ### M125 - Pause print (TODO: not implemented)
    */
    /*!
    ### M25 - Pause SD print <a href="https://reprap.org/wiki/G-code#M25:_Pause_SD_print">M25: Pause SD print</a>
    */
    case 25:
    case 601:
    {
        if (!isPrintPaused) {
            ;//st_synchronize();
            ;//ClearToSend(); //send OK even before the command finishes executing because we want to make sure it is not skipped because of cmdqueue_pop_front();
            cmdqueue_pop_front(); //trick because we want skip this command (M601) after restore
            isPrintPaused = true;//lcd_pause_print();
        }
    }
    break;

    /*!
    ### M602 - Resume print <a href="https://reprap.org/wiki/G-code#M602:_Resume_print">M602: Resume print</a>
    */
    case 602:
    {
        if (isPrintPaused) isPrintPaused = false;//lcd_resume_print();
    }
    break;

    /*!
    ### M603 - Stop print <a href="https://reprap.org/wiki/G-code#M603:_Stop_print">M603: Stop print</a>
    */
    case 603: {
        ;//lcd_print_stop();
    }
    break;

#ifdef PINDA_THERMISTOR
    /*!
	### M860 - Wait for extruder temperature (PINDA) <a href="https://reprap.org/wiki/G-code#M860_Wait_for_Probe_Temperature">M860 Wait for Probe Temperature</a>
    Wait for PINDA thermistor to reach target temperature
    #### Usage
    
        M860 [ S ]
    
    #### Parameters
    - `S` - Target temperature
    */
	case 860: 
	{
		int set_target_pinda = 0;

		if (code_seen('S')) {
			set_target_pinda = code_value_short();
		}
		else {
			break;
		}

		LCD_MESSAGERPGM(_T(MSG_PLEASE_WAIT));

		;//SERIAL_PROTOCOLPGM("Wait for PINDA target temperature:");
		;//SERIAL_PROTOCOLLN(set_target_pinda);

		codenum = _millis();
		cancel_heatup = false;

		bool is_pinda_cooling = false;
		if ((degTargetBed() == 0) && (degTargetHotend(0) == 0)) {
		    is_pinda_cooling = true;
		}

		while ( ((!is_pinda_cooling) && (!cancel_heatup) && (current_temperature_pinda < set_target_pinda)) || (is_pinda_cooling && (current_temperature_pinda > set_target_pinda)) ) {
			if ((_millis() - codenum) > 1000) //Print Temp Reading every 1 second while waiting.
			{
				;//SERIAL_PROTOCOLPGM("P:");
				;//SERIAL_PROTOCOL_F(current_temperature_pinda, 1);
				;//SERIAL_PROTOCOL('/');
				;//SERIAL_PROTOCOLLN(set_target_pinda);
				codenum = _millis();
			}
			manage_heater();
			manage_inactivity();
			lcd_update(0);
		}
		LCD_MESSAGERPGM(MSG_OK);

		break;
	}
 
    /*!
    ### M861 - Set/Get PINDA temperature compensation offsets <a href="https://reprap.org/wiki/G-code#M861_Set_Probe_Thermal_Compensation">M861 Set Probe Thermal Compensation</a>
    Set compensation ustep value `S` for compensation table index `I`.
    #### Usage
    
        M861 [ ? | ! | Z | S | I ]
    
    #### Parameters
    - `?` - Print current EEPROM offset values
    - `!` - Set factory default values
    - `Z` - Set all values to 0 (effectively disabling PINDA temperature compensation)
    - `S` - Microsteps
    - `I` - Table index
    */
	case 861: {
		const char * const _header = PSTR("index, temp, ustep, um");
		if (code_seen('?')) { // ? - Print out current EEPROM offset values
			int16_t usteps = 0;
			;//SERIAL_PROTOCOLPGM("PINDA cal status: ");
			;//SERIAL_PROTOCOLLN(calibration_status_pinda());
			;//SERIAL_PROTOCOLLNRPGM(_header);
			for (uint8_t i = 0; i < 6; i++)
			{
				if(i > 0) {
					usteps = eeprom_read_word((uint16_t*) EEPROM_PROBE_TEMP_SHIFT + (i - 1));
				}
				float mm = ((float)usteps) / cs.axis_steps_per_unit[Z_AXIS];
				i == 0 ? ;//SERIAL_PROTOCOLPGM("n/a") : ;//SERIAL_PROTOCOL(i - 1);
				;//SERIAL_PROTOCOLPGM(", ");
				;//SERIAL_PROTOCOL(35 + (i * 5));
				;//SERIAL_PROTOCOLPGM(", ");
				;//SERIAL_PROTOCOL(usteps);
				;//SERIAL_PROTOCOLPGM(", ");
				;//SERIAL_PROTOCOLLN(mm * 1000);
			}
		}
		else if (code_seen('!')) { // ! - Set factory default values
			eeprom_write_byte((uint8_t*)EEPROM_CALIBRATION_STATUS_PINDA, 1);
			int16_t z_shift = 8;    //40C -  20um -   8usteps
			eeprom_update_word((uint16_t*)EEPROM_PROBE_TEMP_SHIFT, z_shift);
			z_shift = 24;   //45C -  60um -  24usteps
			eeprom_update_word((uint16_t*)EEPROM_PROBE_TEMP_SHIFT + 1, z_shift);
			z_shift = 48;   //50C - 120um -  48usteps
			eeprom_update_word((uint16_t*)EEPROM_PROBE_TEMP_SHIFT + 2, z_shift);
			z_shift = 80;   //55C - 200um -  80usteps
			eeprom_update_word((uint16_t*)EEPROM_PROBE_TEMP_SHIFT + 3, z_shift);
			z_shift = 120;  //60C - 300um - 120usteps
			eeprom_update_word((uint16_t*)EEPROM_PROBE_TEMP_SHIFT + 4, z_shift);
			;//SERIAL_PROTOCOLLNPGM("factory restored");
		}
		else if (code_seen('Z')) { // Z - Set all values to 0 (effectively disabling PINDA temperature compensation)
			eeprom_write_byte((uint8_t*)EEPROM_CALIBRATION_STATUS_PINDA, 1);
			int16_t z_shift = 0;
			for (uint8_t i = 0; i < 5; i++) {
				eeprom_update_word((uint16_t*)EEPROM_PROBE_TEMP_SHIFT + i, z_shift);
			}
			;//SERIAL_PROTOCOLLNPGM("zerorized");
		}
		else if (code_seen('S')) { // Sxxx Iyyy - Set compensation ustep value S for compensation table index I
			int16_t usteps = code_value_short();
			if (code_seen('I')) {
			    uint8_t index = code_value_uint8();
				if (index < 5) {
					eeprom_update_word((uint16_t*)EEPROM_PROBE_TEMP_SHIFT + index, usteps);
					;//SERIAL_PROTOCOLLNRPGM(MSG_OK);
					;//SERIAL_PROTOCOLLNRPGM(_header);
					for (uint8_t i = 0; i < 6; i++)
					{
						usteps = 0;
						if (i > 0) {
							usteps = eeprom_read_word((uint16_t*)EEPROM_PROBE_TEMP_SHIFT + (i - 1));
						}
						float mm = ((float)usteps) / cs.axis_steps_per_unit[Z_AXIS];
						i == 0 ? ;//SERIAL_PROTOCOLPGM("n/a") : ;//SERIAL_PROTOCOL(i - 1);
						;//SERIAL_PROTOCOLPGM(", ");
						;//SERIAL_PROTOCOL(35 + (i * 5));
						;//SERIAL_PROTOCOLPGM(", ");
						;//SERIAL_PROTOCOL(usteps);
						;//SERIAL_PROTOCOLPGM(", ");
						;//SERIAL_PROTOCOLLN(mm * 1000);
					}
				}
			}
		}
		else {
			;//SERIAL_PROTOCOLLNPGM("no valid command");
		}
    } break;

#endif //PINDA_THERMISTOR
   
    /*!
	### M862 - Print checking <a href="https://reprap.org/wiki/G-code#M862:_Print_checking">M862: Print checking</a>
    Checks the parameters of the printer and gcode and performs compatibility check
	
      - M862.1 { P<nozzle_diameter> | Q } 0.25/0.40/0.60
      - M862.2 { P<model_code> | Q }
      - M862.3 { P"<model_name>" | Q }
      - M862.4 { P<fw_version> | Q }
      - M862.5 { P<gcode_level> | Q }
    
    When run with P<> argument, the check is performed against the input value.
    When run with Q argument, the current value is shown.
	
    M862.3 accepts text identifiers of printer types too.
    The syntax of M862.3 is (note the quotes around the type):
	  
          M862.3 P "MK3S"
	  
    Accepted printer type identifiers and their numeric counterparts:
	
      - MK1         (100)
      - MK2         (200)       
      - MK2MM       (201)     
      - MK2S        (202)      
      - MK2SMM      (203)    
      - MK2.5       (250)     
      - MK2.5MMU2   (20250) 
      - MK2.5S      (252)    
      - MK2.5SMMU2S (20252)
      - MK3         (300)
      - MK3MMU2     (20300)
      - MK3S        (302)
      - MK3SMMU2S   (20302)
	
    */
    case 862: // M862: print checking
    {
          float nDummy;
          uint8_t nCommand;
          nCommand=(uint8_t)(modff(code_value_float(),&nDummy)*10.0+0.5);
          switch((ClPrintChecking)nCommand)
               {
               case ClPrintChecking::_Nozzle:     // ~ .1
                    uint16_t nDiameter;
                    if(code_seen('P'))
                         {
                         nDiameter=(uint16_t)(code_value()*1000.0+0.5); // [,um]
                         ;//nozzle_diameter_check(nDiameter);
                         }

                    else if(code_seen('S')&&farm_mode)
                         {
                         nDiameter=(uint16_t)(code_value()*1000.0+0.5); // [,um]
                         ;//eeprom_update_byte((uint8_t*)EEPROM_NOZZLE_DIAMETER,(uint8_t)ClNozzleDiameter::_Diameter_Undef); // for correct synchronization after farm-mode exiting
                         ;//eeprom_update_word((uint16_t*)EEPROM_NOZZLE_DIAMETER_uM,nDiameter);
                         }

                    else if(code_seen('Q'))
                         ;//SERIAL_PROTOCOLLN((float)eeprom_read_word((uint16_t*)EEPROM_NOZZLE_DIAMETER_uM)/1000.0);
                    break;
               case ClPrintChecking::_Model:      // ~ .2
                    if(code_seen('P'))
                         {
                         uint16_t nPrinterModel;
                         nPrinterModel=(uint16_t)code_value_long();
                         ;//printer_model_check(nPrinterModel);
                         }
                    else if(code_seen('Q'))
                         ;//SERIAL_PROTOCOLLN(nPrinterType);
                    break;
               case ClPrintChecking::_Smodel:     // ~ .3
                    if(code_seen('P'))
                         printer_smodel_check(strchr_pointer);
                    else if(code_seen('Q'))
                         ;//SERIAL_PROTOCOLLNRPGM(sPrinterName);
                    break;
               case ClPrintChecking::_Version:    // ~ .4
                    if(code_seen('P'))
                         fw_version_check(++strchr_pointer);
                    else if(code_seen('Q'))
                         ;//SERIAL_PROTOCOLLNRPGM(FW_VERSION_STR_P());
                    break;
               case ClPrintChecking::_Gcode:      // ~ .5
                    if(code_seen('P'))
                         {
                         uint16_t nGcodeLevel;
                         nGcodeLevel=(uint16_t)code_value_long();
                         ;//gcode_level_check(nGcodeLevel);
                         }
                    else if(code_seen('Q'))
                         ;//SERIAL_PROTOCOLLN(GCODE_LEVEL);
                    break;
               }
    }
    break;

#ifdef LIN_ADVANCE
    /*!
	### M900 - Set Linear advance options <a href="https://reprap.org/wiki/G-code#M900_Set_Linear_Advance_Scaling_Factors">M900 Set Linear Advance Scaling Factors</a>
	Sets the advance extrusion factors for Linear Advance. If any of the R, W, H, or D parameters are set to zero the ratio will be computed dynamically during printing.
	#### Usage
    
        M900 [ K | R | W | H | D]
    
    #### Parameters
    - `K` -  Advance K factor
    - `R` - Set ratio directly (overrides WH/D)
    - `W` - Width
    - `H` - Height
    - `D` - Diameter Set ratio from WH/D
    */
    case 900:
    {
        ;//gcode_M900();
        // relevant part copied ->
        float newK = code_seen('K') ? code_value_float() : -2;
    }
    break;
#endif

    /*!
	### M907 - Set digital trimpot motor current in mA using axis codes <a href="https://reprap.org/wiki/G-code#M907:_Set_digital_trimpot_motor">M907: Set digital trimpot motor</a>
	Set digital trimpot motor current using axis codes (X, Y, Z, E, B, S).
    M907 has no effect when the experimental Extruder motor current scaling mode is active (that applies to farm printing as well)
	#### Usage
    
        M907 [ X | Y | Z | E | B | S ]
	
    #### Parameters
    - `X` - X motor driver
    - `Y` - Y motor driver
    - `Z` - Z motor driver
    - `E` - Extruder motor driver
    - `B` - Second Extruder motor driver
    - `S` - All motors
    */
    case 907:
    {
#ifdef TMC2130
        // See tmc2130_cur2val() for translation to 0 .. 63 range
        for (uint_least8_t i = 0; i < NUM_AXIS; i++){
            if(code_seen(axis_codes[i])){
                if( i == E_AXIS && FarmOrUserECool() ){
                    ;//;//SERIAL_ECHORPGM(eMotorCurrentScalingEnabled);
                    ;//;//SERIAL_ECHOLNPGM(", M907 E ignored");
                    continue;
                }
                long cur_mA = code_value_long();
                uint8_t val = tmc2130_cur2val(cur_mA);
                tmc2130_set_current_h(i, val);
                tmc2130_set_current_r(i, val);
                //if (i == E_AXIS) printf_P(PSTR("E-axis current=%ldmA\n"), cur_mA);
            }
        }
#else //TMC2130
      #if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
        for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) st_current_set(i,code_value());
        if(code_seen('B')) st_current_set(4,code_value());
        if(code_seen('S')) for(int i=0;i<=4;i++) st_current_set(i,code_value());
      #endif
      #ifdef MOTOR_CURRENT_PWM_XY_PIN
        if(code_seen('X')) st_current_set(0, code_value());
      #endif
      #ifdef MOTOR_CURRENT_PWM_Z_PIN
        if(code_seen('Z')) st_current_set(1, code_value());
      #endif
      #ifdef MOTOR_CURRENT_PWM_E_PIN
        if(code_seen('E')) st_current_set(2, code_value());
      #endif
#endif //TMC2130
    }
    break;

    /*!
	### M908 - Control digital trimpot directly <a href="https://reprap.org/wiki/G-code#M908:_Control_digital_trimpot_directly">M908: Control digital trimpot directly</a>
	In Prusa Firmware this G-code is deactivated by default, must be turned on in the source code. Not usable on Prusa printers.
    #### Usage
    
        M908 [ P | S ]
    
    #### Parameters
    - `P` - channel
    - `S` - current
    */
    case 908:
    {
      #if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
        uint8_t channel,current;
        if(code_seen('P')) channel=code_value();
        if(code_seen('S')) current=code_value();
        digitalPotWrite(channel, current);
      #endif
    }
    break;

#ifdef TMC2130_SERVICE_CODES_M910_M918

    /*!
	### M910 - TMC2130 init <a href="https://reprap.org/wiki/G-code#M910:_TMC2130_init">M910: TMC2130 init</a>
	Not active in default, only if `TMC2130_SERVICE_CODES_M910_M918` is defined in source code.
	
    */
	case 910:
    {
		tmc2130_init(TMCInitParams(false, FarmOrUserECool()));
    }
    break;

    /*!
    ### M911 - Set TMC2130 holding currents <a href="https://reprap.org/wiki/G-code#M911:_Set_TMC2130_holding_currents">M911: Set TMC2130 holding currents</a>
	Not active in default, only if `TMC2130_SERVICE_CODES_M910_M918` is defined in source code.
    #### Usage
    
        M911 [ X | Y | Z | E ]
    
    #### Parameters
    - `X` - X stepper driver holding current value
    - `Y` - Y stepper driver holding current value
    - `Z` - Z stepper driver holding current value
    - `E` - Extruder stepper driver holding current value
    */
	case 911: 
    {
		if (code_seen('X')) tmc2130_set_current_h(0, code_value());
		if (code_seen('Y')) tmc2130_set_current_h(1, code_value());
        if (code_seen('Z')) tmc2130_set_current_h(2, code_value());
        if (code_seen('E')) tmc2130_set_current_h(3, code_value());
    }
    break;

    /*!
	### M912 - Set TMC2130 running currents <a href="https://reprap.org/wiki/G-code#M912:_Set_TMC2130_running_currents">M912: Set TMC2130 running currents</a>
	Not active in default, only if `TMC2130_SERVICE_CODES_M910_M918` is defined in source code.
    #### Usage
    
        M912 [ X | Y | Z | E ]
    
    #### Parameters
    - `X` - X stepper driver running current value
    - `Y` - Y stepper driver running current value
    - `Z` - Z stepper driver running current value
    - `E` - Extruder stepper driver running current value
    */
	case 912: 
    {
		if (code_seen('X')) tmc2130_set_current_r(0, code_value());
		if (code_seen('Y')) tmc2130_set_current_r(1, code_value());
        if (code_seen('Z')) tmc2130_set_current_r(2, code_value());
        if (code_seen('E')) tmc2130_set_current_r(3, code_value());
    }
    break;

    /*!
	### M913 - Print TMC2130 currents <a href="https://reprap.org/wiki/G-code#M913:_Print_TMC2130_currents">M913: Print TMC2130 currents</a>
	Not active in default, only if `TMC2130_SERVICE_CODES_M910_M918` is defined in source code.
	Shows TMC2130 currents.
    */
	case 913:
    {
		tmc2130_print_currents();
    }
    break;

    /*!
	### M914 - Set TMC2130 normal mode <a href="https://reprap.org/wiki/G-code#M914:_Set_TMC2130_normal_mode">M914: Set TMC2130 normal mode</a>
	Not active in default, only if `TMC2130_SERVICE_CODES_M910_M918` is defined in source code.
    */
    case 914:
    {
		tmc2130_mode = TMC2130_MODE_NORMAL;
		update_mode_profile();
		tmc2130_init(TMCInitParams(false, FarmOrUserECool()));
    }
    break;

    /*!
	### M915 - Set TMC2130 silent mode <a href="https://reprap.org/wiki/G-code#M915:_Set_TMC2130_silent_mode">M915: Set TMC2130 silent mode</a>
	Not active in default, only if `TMC2130_SERVICE_CODES_M910_M918` is defined in source code.
    */
    case 915:
    {
		tmc2130_mode = TMC2130_MODE_SILENT;
		update_mode_profile();
		tmc2130_init(TMCInitParams(false, FarmOrUserECool()));
    }
    break;

    /*!
	### M916 - Set TMC2130 Stallguard sensitivity threshold <a href="https://reprap.org/wiki/G-code#M916:_Set_TMC2130_Stallguard_sensitivity_threshold">M916: Set TMC2130 Stallguard sensitivity threshold</a>
	Not active in default, only if `TMC2130_SERVICE_CODES_M910_M918` is defined in source code.
    #### Usage
    
        M916 [ X | Y | Z | E ]
    
    #### Parameters
    - `X` - X stepper driver stallguard sensitivity threshold value
    - `Y` - Y stepper driver stallguard sensitivity threshold value
    - `Z` - Z stepper driver stallguard sensitivity threshold value
    - `E` - Extruder stepper driver stallguard sensitivity threshold value
    */
	case 916:
    {
		if (code_seen('X')) tmc2130_sg_thr[X_AXIS] = code_value();
		if (code_seen('Y')) tmc2130_sg_thr[Y_AXIS] = code_value();
		if (code_seen('Z')) tmc2130_sg_thr[Z_AXIS] = code_value();
		if (code_seen('E')) tmc2130_sg_thr[E_AXIS] = code_value();
		for (uint8_t a = X_AXIS; a <= E_AXIS; a++)
			printf_P(_N("tmc2130_sg_thr[%c]=%d\n"), "XYZE"[a], tmc2130_sg_thr[a]);
    }
    break;

    /*!
	### M917 - Set TMC2130 PWM amplitude offset (pwm_ampl) <a href="https://reprap.org/wiki/G-code#M917:_Set_TMC2130_PWM_amplitude_offset_.28pwm_ampl.29">M917: Set TMC2130 PWM amplitude offset (pwm_ampl)</a>
	Not active in default, only if `TMC2130_SERVICE_CODES_M910_M918` is defined in source code.
    #### Usage
    
        M917 [ X | Y | Z | E ]
    
    #### Parameters
    - `X` - X stepper driver PWM amplitude offset value
    - `Y` - Y stepper driver PWM amplitude offset value
    - `Z` - Z stepper driver PWM amplitude offset value
    - `E` - Extruder stepper driver PWM amplitude offset value
    */
	case 917:
    {
		if (code_seen('X')) tmc2130_set_pwm_ampl(0, code_value());
		if (code_seen('Y')) tmc2130_set_pwm_ampl(1, code_value());
        if (code_seen('Z')) tmc2130_set_pwm_ampl(2, code_value());
        if (code_seen('E')) tmc2130_set_pwm_ampl(3, code_value());
    }
    break;

    /*!
	### M918 - Set TMC2130 PWM amplitude gradient (pwm_grad) <a href="https://reprap.org/wiki/G-code#M918:_Set_TMC2130_PWM_amplitude_gradient_.28pwm_grad.29">M918: Set TMC2130 PWM amplitude gradient (pwm_grad)</a>
	Not active in default, only if `TMC2130_SERVICE_CODES_M910_M918` is defined in source code.
    #### Usage
    
        M918 [ X | Y | Z | E ]
    
    #### Parameters
    - `X` - X stepper driver PWM amplitude gradient value
    - `Y` - Y stepper driver PWM amplitude gradient value
    - `Z` - Z stepper driver PWM amplitude gradient value
    - `E` - Extruder stepper driver PWM amplitude gradient value
    */
	case 918:
    {
		if (code_seen('X')) tmc2130_set_pwm_grad(0, code_value());
		if (code_seen('Y')) tmc2130_set_pwm_grad(1, code_value());
        if (code_seen('Z')) tmc2130_set_pwm_grad(2, code_value());
        if (code_seen('E')) tmc2130_set_pwm_grad(3, code_value());
    }
    break;

#endif //TMC2130_SERVICE_CODES_M910_M918

    /*!
	### M350 - Set microstepping mode <a href="https://reprap.org/wiki/G-code#M350:_Set_microstepping_mode">M350: Set microstepping mode</a>
    Printers with TMC2130 drivers have `X`, `Y`, `Z` and `E` as options. The steps-per-unit value is updated accordingly. Not all resolutions are valid!
    Printers without TMC2130 drivers also have `B` and `S` options. In this case, the steps-per-unit value in not changed!
    #### Usage
    
        M350 [ X | Y | Z | E | B | S ]
    
    #### Parameters
    - `X` - X new resolution
    - `Y` - Y new resolution
    - `Z` - Z new resolution
    - `E` - E new resolution
    
    Only valid for MK2.5(S) or printers without TMC2130 drivers
    - `B` - Second extruder new resolution
    - `S` - All axes new resolution
    */
    case 350: 
    {
	#ifdef TMC2130
		for (uint_least8_t i=0; i<NUM_AXIS; i++) 
		{
			if(code_seen(axis_codes[i]))
			{
				uint16_t res_new = code_value();
#ifdef ALLOW_ALL_MRES
				bool res_valid = res_new > 0 && res_new <= 256 && !(res_new & (res_new - 1)); // must be a power of two
#else
				bool res_valid = (res_new == 8) || (res_new == 16) || (res_new == 32); // resolutions valid for all axis
				res_valid |= (i != E_AXIS) && ((res_new == 1) || (res_new == 2) || (res_new == 4)); // resolutions valid for X Y Z only
				res_valid |= (i == E_AXIS) && ((res_new == 64) || (res_new == 128)); // resolutions valid for E only
#endif
				if (res_valid)
				{
					st_synchronize();
					uint16_t res = tmc2130_get_res(i);
					tmc2130_set_res(i, res_new);
					cs.axis_ustep_resolution[i] = res_new;
					if (res_new > res)
					{
						uint16_t fac = (res_new / res);
						cs.axis_steps_per_unit[i] *= fac;
						position[i] *= fac;
					}
					else
					{
						uint16_t fac = (res / res_new);
						cs.axis_steps_per_unit[i] /= fac;
						position[i] /= fac;
					}
#if defined(FILAMENT_SENSOR) && defined(PAT9125)
                    if (i == E_AXIS)
                        fsensor_set_axis_steps_per_unit(cs.axis_steps_per_unit[i]);
#endif
				}
			}
		}
		reset_acceleration_rates();
	#else //TMC2130
      #if defined(X_MS1_PIN) && X_MS1_PIN > -1
        if(code_seen('S')) for(int i=0;i<=4;i++) microstep_mode(i,code_value());
        for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_mode(i,(uint8_t)code_value());
        if(code_seen('B')) microstep_mode(4,code_value());
        microstep_readings();
      #endif
	#endif //TMC2130
    }
    break;

    /*!
	### M351 - Toggle Microstep Pins <a href="https://reprap.org/wiki/G-code#M351:_Toggle_MS1_MS2_pins_directly">M351: Toggle MS1 MS2 pins directly</a>
    Toggle MS1 MS2 pins directly.
    #### Usage
    
        M351 [B<0|1>] [E<0|1>] S<1|2> [X<0|1>] [Y<0|1>] [Z<0|1>]
    
    #### Parameters
    - `X` - Update X axis
    - `Y` - Update Y axis
    - `Z` - Update Z axis
    - `E` - Update E axis
    - `S` - which MSx pin to toggle
    - `B` - new pin value
    */
    case 351:
    {
      #if defined(X_MS1_PIN) && X_MS1_PIN > -1
      if(code_seen('S')) switch((int)code_value())
      {
        case 1:
          for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_ms(i,code_value(),-1);
          if(code_seen('B')) microstep_ms(4,code_value(),-1);
          break;
        case 2:
          for(int i=0;i<NUM_AXIS;i++) if(code_seen(axis_codes[i])) microstep_ms(i,-1,code_value());
          if(code_seen('B')) microstep_ms(4,-1,code_value());
          break;
      }
      microstep_readings();
      #endif
    }
    break;

  /*!
  ### M701 - Load filament <a href="https://reprap.org/wiki/G-code#M701:_Load_filament">M701: Load filament</a>
  
  */
	case 701:
	{
		if (true/*mmu_enabled*/ && code_seen('E'))
			/*tmp_extruder = */code_value_uint8();
		;//gcode_M701();
	}
	break;

    /*!
    ### M702 - Unload filament <a href="https://reprap.org/wiki/G-code#M702:_Unload_filament">G32: Undock Z Probe sled</a>
    #### Usage
    
        M702 [ U | C ]
    
    #### Parameters
    - `C` - Unload just current filament
    - without any parameters unload all filaments
    */
	case 702:
	{
		if (code_seen('C')) {
			if(true/*mmu_enabled*/) 
      ;//extr_unload(); //! if "C" unload current filament; if mmu is not present no action is performed
		}
		else {
			if(true/*mmu_enabled*/) 
      ;//extr_unload(); //! unload current filament
			else 
      ;//unload_filament();
		}
	}
	break;

    /*!
    ### M999 - Restart after being stopped <a href="https://reprap.org/wiki/G-code#M999:_Restart_after_being_stopped_by_error">M999: Restart after being stopped by error</a>
    @todo Usually doesn't work. Should be fixed or removed. Most of the time, if `Stopped` it set, the print fails and is unrecoverable.
    */
    case 999:
      Stopped = false;
      ;//lcd_reset_alert_level();
      gcode_LastN = Stopped_gcode_LastN;
      ;//FlushSerialRequestResend();
    break;
	/*!
	#### End of M-Commands
    */
	default: 
		;//printf_P(MSG_UNKNOWN_CODE, 'M', cmdbuffer + bufindr + CMDHDRSIZE);
    }
//	printf_P(_N("END M-CODE=%u\n"), mcode_in_progress);
	mcode_in_progress = 0;
  }
  }
  // end if(code_seen('M')) (end of M codes)
  /*!
  -----------------------------------------------------------------------------------------
  # T Codes
  T<extruder nr.> - select extruder in case of multi extruder printer. select filament in case of MMU_V2.
  #### For MMU_V2:
  T<n> Gcode to extrude at least 38.10 mm at feedrate 19.02 mm/s must follow immediately to load to extruder wheels.
  @n T? Gcode to extrude shouldn't have to follow, load to extruder wheels is done automatically
  @n Tx Same as T?, except nozzle doesn't have to be preheated. Tc must be placed after extruder nozzle is preheated to finish filament load.
  @n Tc Load to nozzle after filament was prepared by Tc and extruder nozzle is already heated.
  */
  else if(code_seen('T'))
  {
      static const char duplicate_Tcode_ignored[] = "Duplicate T-code ignored.";
      
      int index;
      bool load_to_nozzle = false;
      for (index = 1; *(strchr_pointer + index) == ' ' || *(strchr_pointer + index) == '\t'; index++);

      *(strchr_pointer + index) = tolower(*(strchr_pointer + index));

      if ((*(strchr_pointer + index) < '0' || *(strchr_pointer + index) > '4') && *(strchr_pointer + index) != '?' && *(strchr_pointer + index) != 'x' && *(strchr_pointer + index) != 'c') {
          ;//;//SERIAL_ECHOLNPGM("Invalid T code.");
      }
	  else if (*(strchr_pointer + index) == 'x'){ //load to bondtech gears; if mmu is not present do nothing
		if (true/*mmu_enabled*/)
		{
      ;//
			/* tmp_extruder = choose_menu_P(_T(MSG_CHOOSE_FILAMENT), _T(MSG_FILAMENT));
			if ((tmp_extruder == mmu_extruder) && mmu_fil_loaded) //dont execute the same T-code twice in a row
			{
				puts_P(duplicate_Tcode_ignored);
			}
			else
			{
				st_synchronize();
				mmu_command(MmuCmd::T0 + tmp_extruder);
				manage_response(true, true, MMU_TCODE_MOVE);
			} */
		}
	  }
	  else if (*(strchr_pointer + index) == 'c') { //load to from bondtech gears to nozzle (nozzle should be preheated)
	  	if (true/*mmu_enabled*/) 
		{
			;//st_synchronize();
			;//mmu_continue_loading(usb_timer.running()  || (lcd_commands_type == LcdCommands::Layer1Cal));
			;//mmu_extruder = tmp_extruder; //filament change is finished
			;//mmu_load_to_nozzle();
		}
	  }
      else {
          if (*(strchr_pointer + index) == '?')
          {
              if(true/*mmu_enabled*/)
              {
                  ;//tmp_extruder = choose_menu_P(_T(MSG_CHOOSE_FILAMENT), _T(MSG_FILAMENT));
                  load_to_nozzle = true;
              } else
              {
                  ;//tmp_extruder = choose_menu_P(_T(MSG_CHOOSE_EXTRUDER), _T(MSG_EXTRUDER));
              }
          }
          else {
              ;//tmp_extruder = code_value();
              if (true/*mmu_enabled*/ /*&& lcd_autoDepleteEnabled()*/)
              {
                  ;//tmp_extruder = ad_getAlternative(tmp_extruder);
              }
          }
          ;//st_synchronize();

          if (true/*mmu_enabled*/)
          {
              if (true/*(tmp_extruder == mmu_extruder) && mmu_fil_loaded*/) //dont execute the same T-code twice in a row
              {
                  ;//puts_P(duplicate_Tcode_ignored);
              }
			  else
			  {
#if defined(MMU_HAS_CUTTER) && defined(MMU_ALWAYS_CUT)
			      if (EEPROM_MMU_CUTTER_ENABLED_always == eeprom_read_byte((uint8_t*)EEPROM_MMU_CUTTER_ENABLED))
                  {
                      mmu_command(MmuCmd::K0 + tmp_extruder);
                      manage_response(true, true, MMU_UNLOAD_MOVE);
                  }
#endif //defined(MMU_HAS_CUTTER) && defined(MMU_ALWAYS_CUT)
				  ;//mmu_command(MmuCmd::T0 + tmp_extruder);
				  ;//manage_response(true, true, MMU_TCODE_MOVE);
		          ;//mmu_continue_loading(usb_timer.running()  || (lcd_commands_type == LcdCommands::Layer1Cal));

				  ;//mmu_extruder = tmp_extruder; //filament change is finished

				  if (load_to_nozzle)// for single material usage with mmu
				  {
					  //mmu_load_to_nozzle();
				  }
			  }
          }
          else
          {
              if (true/*tmp_extruder >= EXTRUDERS*/) {
                  ;//;//SERIAL_ECHO_START;
                  ;//;//SERIAL_ECHO('T');
                  ;//SERIAL_PROTOCOLLN((int)tmp_extruder);
                  ;//;//SERIAL_ECHOLNRPGM(_n("Invalid extruder"));////MSG_INVALID_EXTRUDER
              }
              else {
#if EXTRUDERS > 1
                  bool make_move = false;
#endif
                  if (code_seen('F')) {
#if EXTRUDERS > 1
                      make_move = true;
#endif
                      next_feedrate = code_value();
                      if (next_feedrate > 0.0) {
                          feedrate = next_feedrate;
                      }
                  }
#if EXTRUDERS > 1
                  if (tmp_extruder != active_extruder) {
                      // Save current position to return to after applying extruder offset
                      set_destination_to_current();
                      // Offset extruder (only by XY)
                      int i;
                      for (i = 0; i < 2; i++) {
                          current_position[i] = current_position[i] -
                                  extruder_offset[i][active_extruder] +
                                  extruder_offset[i][tmp_extruder];
                      }
                      // Set the new active extruder and position
                      active_extruder = tmp_extruder;
                      plan_set_position_curposXYZE();
                      // Move to the old position if 'F' was in the parameters
                      if (make_move && Stopped == false) {
                          prepare_move();
                      }
                  }
#endif
                  ;//;//SERIAL_ECHO_START;
                  ;//;//SERIAL_ECHORPGM(_n("Active Extruder: "));////MSG_ACTIVE_EXTRUDER
                  ;//SERIAL_PROTOCOLLN((int)active_extruder);
              }
          }
      }
  } // end if(code_seen('T')) (end of T codes)
  /*!
  #### End of T-Codes
  */

  /**
  *---------------------------------------------------------------------------------
  *# D codes
  */
  else if (code_seen('D')) // D codes (debug)
  {
    switch(code_value_short())
    {

    /*!
    ### D-1 - Endless Loop <a href="https://reprap.org/wiki/G-code#D-1:_Endless_Loop">D-1: Endless Loop</a>
    */
	case -1:
		;//dcode__1(); -- We Don't want an endless loop when fuzzing 
    break;
#ifdef DEBUG_DCODES

    /*!
    ### D0 - Reset <a href="https://reprap.org/wiki/G-code#D0:_Reset">D0: Reset</a>
    #### Usage
    
        D0 [ B ]
    
    #### Parameters
    - `B` - Bootloader
    */
	case 0:
		dcode_0(); break;

    /*!
    *
    ### D1 - Clear EEPROM and RESET <a href="https://reprap.org/wiki/G-code#D1:_Clear_EEPROM_and_RESET">D1: Clear EEPROM and RESET</a>
      
          D1
      
    *
    */
	case 1:
		dcode_1(); break;
#endif

#if defined DEBUG_DCODE2 || defined DEBUG_DCODES
    /*!
    ### D2 - Read/Write RAM <a href="https://reprap.org/wiki/G-code#D2:_Read.2FWrite_RAM">D3: Read/Write RAM</a>
    This command can be used without any additional parameters. It will read the entire RAM.
    #### Usage
    
        D2 [ A | C | X ]
    
    #### Parameters
    - `A` - Address (x0000-x1fff)
    - `C` - Count (1-8192)
    - `X` - Data

	#### Notes
	- The hex address needs to be lowercase without the 0 before the x
	- Count is decimal 
	- The hex data needs to be lowercase
	
    */
	case 2:
		dcode_2();
    break;
#endif //DEBUG_DCODES
#if defined DEBUG_DCODE3 || defined DEBUG_DCODES

    /*!
    ### D3 - Read/Write EEPROM <a href="https://reprap.org/wiki/G-code#D3:_Read.2FWrite_EEPROM">D3: Read/Write EEPROM</a>
    This command can be used without any additional parameters. It will read the entire eeprom.
    #### Usage
    
        D3 [ A | C | X ]
    
    #### Parameters
    - `A` - Address (x0000-x0fff)
    - `C` - Count (1-4096)
    - `X` - Data (hex)
	
	#### Notes
	- The hex address needs to be lowercase without the 0 before the x
	- Count is decimal 
	- The hex data needs to be lowercase
	
    */
	case 3:
		dcode_2(); break;
#endif //DEBUG_DCODE3
#ifdef DEBUG_DCODES

    /*!
    
    ### D4 - Read/Write PIN <a href="https://reprap.org/wiki/G-code#D4:_Read.2FWrite_PIN">D4: Read/Write PIN</a>
    To read the digital value of a pin you need only to define the pin number.
    #### Usage
    
        D4 [ P | F | V ]
    
    #### Parameters
    - `P` - Pin (0-255)
    - `F` - Function in/out (0/1)
    - `V` - Value (0/1)
    */
	case 4:
		dcode_4(); break;
#endif //DEBUG_DCODES
#if defined DEBUG_DCODE5 || defined DEBUG_DCODES

    /*!
    ### D5 - Read/Write FLASH <a href="https://reprap.org/wiki/G-code#D5:_Read.2FWrite_FLASH">D5: Read/Write Flash</a>
    This command can be used without any additional parameters. It will read the 1kb FLASH.
    #### Usage
    
        D5 [ A | C | X | E ]
    
    #### Parameters
    - `A` - Address (x00000-x3ffff)
    - `C` - Count (1-8192)
    - `X` - Data (hex)
    - `E` - Erase
 	
	#### Notes
	- The hex address needs to be lowercase without the 0 before the x
	- Count is decimal 
	- The hex data needs to be lowercase
	
   */
	case 5:
		dcode_5(); break;
#endif //DEBUG_DCODE5
#if defined DEBUG_DCODE6 || defined DEBUG_DCODES

    /*!
    ### D6 - Read/Write external FLASH <a href="https://reprap.org/wiki/G-code#D6:_Read.2FWrite_external_FLASH">D6: Read/Write external Flash</a>
    Reserved
    */
	case 6:
		dcode_2(); break;
#endif
#ifdef DEBUG_DCODES

    /*!
    ### D7 - Read/Write Bootloader <a href="https://reprap.org/wiki/G-code#D7:_Read.2FWrite_Bootloader">D7: Read/Write Bootloader</a>
    Reserved
    */
	case 7:
		dcode_7(); break;

    /*!
    ### D8 - Read/Write PINDA <a href="https://reprap.org/wiki/G-code#D8:_Read.2FWrite_PINDA">D8: Read/Write PINDA</a>
    #### Usage
    
        D8 [ ? | ! | P | Z ]
    
    #### Parameters
    - `?` - Read PINDA temperature shift values
    - `!` - Reset PINDA temperature shift values to default
    - `P` - Pinda temperature [C]
    - `Z` - Z Offset [mm]
    */
	case 8:
		dcode_8(); break;

    /*!
    ### D9 - Read ADC <a href="https://reprap.org/wiki/G-code#D9:_Read.2FWrite_ADC">D9: Read ADC</a>
    #### Usage
    
        D9 [ I | V ]
    
    #### Parameters
    - `I` - ADC channel index 
        - `0` - Heater 0 temperature
        - `1` - Heater 1 temperature
        - `2` - Bed temperature
        - `3` - PINDA temperature
        - `4` - PWR voltage
        - `5` - Ambient temperature
        - `6` - BED voltage
    - `V` Value to be written as simulated
    */
	case 9:
		dcode_9(); break;

    /*!
    ### D10 - Set XYZ calibration = OK <a href="https://reprap.org/wiki/G-code#D10:_Set_XYZ_calibration_.3D_OK">D10: Set XYZ calibration = OK</a>
    */
	case 10:
		dcode_10(); break;

    /*!
    ### D12 - Time <a href="https://reprap.org/wiki/G-code#D12:_Time">D12: Time</a>
    Writes the current time in the log file.
    */
#endif //DEBUG_DCODES

#ifdef XFLASH_DUMP
    /*!
    ### D20 - Generate an offline crash dump <a href="https://reprap.org/wiki/G-code#D20:_Generate_an_offline_crash_dump">D20: Generate an offline crash dump</a>
    Generate a crash dump for later retrival.
    #### Usage

     D20 [E]

    ### Parameters
    - `E` - Perform an emergency crash dump (resets the printer).
    ### Notes
    - A crash dump can be later recovered with D21, or cleared with D22.
    - An emergency crash dump includes register data, but will cause the printer to reset after the dump
      is completed.
    */
    case 20: {
        ;//dcode_20();
        break;
    };

    /*!
    ### D21 - Print crash dump to serial <a href="https://reprap.org/wiki/G-code#D21:_Print_crash_dump_to_serial">D21: Print crash dump to serial</a>
    Output the complete crash dump (if present) to the serial.
    #### Usage

     D21

    ### Notes
    - The starting address can vary between builds, but it's always at the beginning of the data section.
    */
    case 21: {
        ;//dcode_21();
        break;
    };

    /*!
    ### D22 - Clear crash dump state <a href="https://reprap.org/wiki/G-code#D22:_Clear_crash_dump_state">D22: Clear crash dump state</a>
    Clear an existing internal crash dump.
    #### Usage

     D22
    */
    case 22: {
        ;//dcode_22();
        break;
    };
#endif //XFLASH_DUMP

#ifdef EMERGENCY_SERIAL_DUMP
    /*!
    ### D23 - Request emergency dump on serial <a href="https://reprap.org/wiki/G-code#D23:_Request_emergency_dump_on_serial">D23: Request emergency dump on serial</a>
    On boards without offline dump support, request online dumps to the serial port on firmware faults.
    When online dumps are enabled, the FW will dump memory on the serial before resetting.
    #### Usage

     D23 [E] [R]
    #### Parameters
    - `E` - Perform an emergency crash dump (resets the printer).
    - `R` - Disable online dumps.
    */
    case 23: {
        ;//dcode_23();
        break;
    };
#endif

#ifdef HEATBED_ANALYSIS

    /*!
    ### D80 - Bed check <a href="https://reprap.org/wiki/G-code#D80:_Bed_check">D80: Bed check</a>
    This command will log data to SD card file "mesh.txt".
    #### Usage
    
        D80 [ E | F | G | H | I | J ]
    
    #### Parameters
    - `E` - Dimension X (default 40)
    - `F` - Dimention Y (default 40)
    - `G` - Points X (default 40)
    - `H` - Points Y (default 40)
    - `I` - Offset X (default 74)
    - `J` - Offset Y (default 34)
  */
	case 80:
		dcode_80(); break;

    /*!
    ### D81 - Bed analysis <a href="https://reprap.org/wiki/G-code#D81:_Bed_analysis">D80: Bed analysis</a>
    This command will log data to SD card file "wldsd.txt".
    #### Usage
    
        D81 [ E | F | G | H | I | J ]
    
    #### Parameters
    - `E` - Dimension X (default 40)
    - `F` - Dimention Y (default 40)
    - `G` - Points X (default 40)
    - `H` - Points Y (default 40)
    - `I` - Offset X (default 74)
    - `J` - Offset Y (default 34)
  */
	case 81:
		dcode_81(); break;
	
#endif //HEATBED_ANALYSIS
#ifdef DEBUG_DCODES

    /*!
    ### D106 - Print measured fan speed for different pwm values <a href="https://reprap.org/wiki/G-code#D106:_Print_measured_fan_speed_for_different_pwm_values">D106: Print measured fan speed for different pwm values</a>
    */
	case 106:
		dcode_106(); break;

#ifdef TMC2130
    /*!
    ### D2130 - Trinamic stepper controller <a href="https://reprap.org/wiki/G-code#D2130:_Trinamic_stepper_controller">D2130: Trinamic stepper controller</a>
    @todo Please review by owner of the code. RepRap Wiki Gcode needs to be updated after review of owner as well.
    
    #### Usage
    
        D2130 [ Axis | Command | Subcommand | Value ]
    
    #### Parameters
    - Axis
      - `X` - X stepper driver
      - `Y` - Y stepper driver
      - `Z` - Z stepper driver
      - `E` - Extruder stepper driver
    - Commands
      - `0`   - Current off
      - `1`   - Current on
      - `+`   - Single step
      - `-`   - Single step oposite direction
      - `NNN` - Value sereval steps
      - `?`   - Read register
      - Subcommands for read register
        - `mres`     - Micro step resolution. More information in datasheet '5.5.2 CHOPCONF – Chopper Configuration'
        - `step`     - Step
        - `mscnt`    - Microstep counter. More information in datasheet '5.5 Motor Driver Registers'
        - `mscuract` - Actual microstep current for motor. More information in datasheet '5.5 Motor Driver Registers'
        - `wave`     - Microstep linearity compensation curve
      - `!`   - Set register
      - Subcommands for set register
        - `mres`     - Micro step resolution
        - `step`     - Step
        - `wave`     - Microstep linearity compensation curve
        - Values for set register
          - `0, 180 --> 250` - Off
          - `0.9 --> 1.25`   - Valid values (recommended is 1.1)
      - `@`   - Home calibrate axis
    
    Examples:
      
          D2130E?wave
      
      Print extruder microstep linearity compensation curve
      
          D2130E!wave0
      
      Disable extruder linearity compensation curve, (sine curve is used)
      
          D2130E!wave220
      
      (sin(x))^1.1 extruder microstep compensation curve used
    
    Notes:
      For more information see https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC2130_datasheet.pdf
    *
	*/
	case 2130:
		dcode_2130(); break;
#endif //TMC2130

#if (defined (FILAMENT_SENSOR) && defined(PAT9125))

    /*!
    ### D9125 - PAT9125 filament sensor <a href="https://reprap.org/wiki/G-code#D9:_Read.2FWrite_ADC">D9125: PAT9125 filament sensor</a>
    #### Usage
    
        D9125 [ ? | ! | R | X | Y | L ]
    
    #### Parameters
    - `?` - Print values
    - `!` - Print values
    - `R` - Resolution. Not active in code
    - `X` - X values
    - `Y` - Y values
    - `L` - Activate filament sensor log
    */
	case 9125:
		dcode_9125(); break;
#endif //FILAMENT_SENSOR

#endif //DEBUG_DCODES

    default:
        ;//printf_P(MSG_UNKNOWN_CODE, 'D', cmdbuffer + bufindr + CMDHDRSIZE);
	}
  }

  else
  {
    ;//;//SERIAL_ECHO_START;
    ;//;//SERIAL_ECHORPGM(MSG_UNKNOWN_COMMAND);
    ;//;//SERIAL_ECHO(CMDBUFFER_CURRENT_STRING);
    ;//;//SERIAL_ECHOLNPGM("\"(2)");
  }
  ;//KEEPALIVE_STATE(NOT_BUSY);
  ;//ClearToSend();

    #if DEBUG_OUTPUT
      std::cout << "\n";
    #endif

}