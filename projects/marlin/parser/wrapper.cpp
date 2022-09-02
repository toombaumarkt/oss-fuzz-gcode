#include <iostream>
#include <sstream>
#include <string.h>
#include <stdio.h>
#include <vector>

#include "wrapper.h"
#include "parser.h"
#include "queue.h"

bool SizeIsOk(size_t Size) {
    return Size >= MIN_DATA_SIZE && Size <= MAX_DATA_SIZE;
}

std::stringstream ConvertByteDataToStringStream(const uint8_t *Data, size_t Size) {
    char* fuzzed_input = new char[Size + 1];
    memcpy(fuzzed_input, Data, Size);
    fuzzed_input[Size] = '\0';
    std::stringstream ss(fuzzed_input);
    delete[]fuzzed_input;
    return ss;
}

#define PS_NORMAL 0
#define PS_EOL    1
#define PS_QUOTED 2
#define PS_PAREN  3
#define PS_ESC    4

/**
 * Copied from queue.cpp
 *
 * Handle a line being completed. For an empty line
 * keep sensor readings going and watchdog alive.
 */
inline bool process_line_done(uint8_t &sis, char (&buff)[MAX_CMD_SIZE], int &ind) {
  sis = PS_NORMAL;                    // "Normal" Serial Input State
  buff[ind] = '\0';                   // Of course, I'm a Terminator.
  const bool is_empty = (ind == 0);   // An empty line?
  if (is_empty)
    int a;//thermalManager.manage_heater();   // Keep sensors satisfied
  else
    ind = 0;                          // Start a new line
  return is_empty;                    // Inform the caller
}

inline void process_stream_char(const char c, uint8_t &sis, char (&buff)[MAX_CMD_SIZE], int &ind) {

  if (sis == PS_EOL) return;    // EOL comment or overflow

  #if ENABLED(PAREN_COMMENTS)
    else if (sis == PS_PAREN) { // Inline comment
      if (c == ')') sis = PS_NORMAL;
      return;
    }
  #endif

  else if (sis >= PS_ESC)       // End escaped char
    sis -= PS_ESC;

  else if (c == '\\') {         // Start escaped char
    sis += PS_ESC;
    if (sis == PS_ESC) return;  // Keep if quoting
  }

  #if ENABLED(GCODE_QUOTED_STRINGS)

    else if (sis == PS_QUOTED) {
      if (c == '"') sis = PS_NORMAL; // End quoted string
    }
    else if (c == '"')          // Start quoted string
      sis = PS_QUOTED;

  #endif

  else if (c == ';') {          // Start end-of-line comment
    sis = PS_EOL;
    return;
  }

  #if ENABLED(PAREN_COMMENTS)
    else if (c == '(') {        // Start inline comment
      sis = PS_PAREN;
      return;
    }
  #endif

  // Backspace erases previous characters
  if (c == 0x08) {
    if (ind) buff[--ind] = '\0';
  }
  else {
    buff[ind++] = c;
    if (ind >= MAX_CMD_SIZE - 1)
      sis = PS_EOL;             // Skip the rest on overflow
  }
}

void simulate_serial_transmission(const uint8_t * Data, size_t length) {
    int index = 0;
    long last_N = 0;
    GCodeQueue::SerialState serial =  {0};

    for (int index = 0; index < length; index++) {
    // Unless a serial port has data, this will exit on next iteration
    //hadData = false;

      // Check if the queue is full and exit if it is.
      //if (ring_buffer.full()) return;

      // No data for this port ? Skip it
      //if (index >= length) return;

      // Ok, we have some data to process, let's make progress here
      //hadData = true;

      const int c = Data[index];
      if (c < 0) {
        // This should never happen, let's log it
        //PORT_REDIRECT(SERIAL_PORTMASK(p));     // Reply to the serial port that sent the command
        // Crash here to get more information why it failed
        //BUG_ON("SP available but read -1");
        //SERIAL_ERROR_MSG(STR_ERR_SERIAL_MISMATCH);
        //SERIAL_FLUSH();
        continue;
      }

      const char serial_char = (char)c;

      if (ISEOL(serial_char)) {

        // Reset our state, continue if the line was empty
        if (process_line_done(serial.input_state, serial.line_buffer, serial.count))
          continue;

        char* command = serial.line_buffer;

        while (*command == ' ') command++;                   // Skip leading spaces
        char *npos = (*command == 'N') ? command : nullptr;  // Require the N parameter to start the line

        if (npos) {

          const bool M110 = !!strstr(command, "M110");

          if (M110) {
            char* n2pos = strchr(command + 4, 'N');
            if (n2pos) npos = n2pos;
          }

          const long gcode_N = strtol(npos + 1, nullptr, 10);

          if (gcode_N != last_N + 1 && !M110) {
            // In case of error on a serial port, don't prevent other serial port from making progress
            //gcode_line_error(F(STR_ERR_LINE_NO), p);
            #if not defined(FUZZING_BUILD_MODE_UNSAFE_FOR_PRODUCTION)
                break;
              #endif
          }

          char *apos = strrchr(command, '*');
          if (apos) {
            uint8_t checksum = 0, count = uint8_t(apos - command);
            while (count) checksum ^= command[--count];
            if (strtol(apos + 1, nullptr, 10) != checksum) {
              // In case of error on a serial port, don't prevent other serial port from making progress
              //gcode_line_error(F(STR_ERR_CHECKSUM_MISMATCH), p);
              #if not defined(FUZZING_BUILD_MODE_UNSAFE_FOR_PRODUCTION)
                break;
              #endif
            }
          }
          else {
            // In case of error on a serial port, don't prevent other serial port from making progress
            //gcode_line_error(F(STR_ERR_NO_CHECKSUM), p);
            #if not defined(FUZZING_BUILD_MODE_UNSAFE_FOR_PRODUCTION)
              break;
            #endif
          }

          last_N = gcode_N;
        }
        #if ENABLED(SDSUPPORT)
          // Pronterface "M29" and "M29 " has no line number
          else if (card.flag.saving && !is_M29(command)) {
            gcode_line_error(F(STR_ERR_NO_CHECKSUM), p);
            break;
          }
        #endif

        //
        // Movement commands give an alert when the machine is stopped
        //

        if (1) {
          char* gpos = strchr(command, 'G');
          if (gpos) {
            switch (strtol(gpos + 1, nullptr, 10)) {
              case 0 ... 1:
              TERN_(ARC_SUPPORT, case 2 ... 3:)
              TERN_(BEZIER_CURVE_SUPPORT, case 5:)
                //PORT_REDIRECT(SERIAL_PORTMASK(p));     // Reply to the serial port that sent the command
                //SERIAL_ECHOLNPGM(STR_ERR_STOPPED);
                //LCD_MESSAGE(MSG_STOPPED);
                break;
            }
          }
        }

        #if DISABLED(EMERGENCY_PARSER)
          // Process critical commands early
          if (command[0] == 'M') switch (command[3]) {
            case '8': if (command[2] == '0' && command[1] == '1') /*{ wait_for_heatup = false; TERN_(HAS_MARLINUI_MENU, wait_for_user = false); }*/ break;
            case '2': if (command[2] == '1' && command[1] == '1') /*kill(FPSTR(M112_KILL_STR), nullptr, true);*/ break;
            case '0': if (command[1] == '4' && command[2] == '1') /*quickstop_stepper();*/ break;
          }
        #endif

        //GCodeQueue::ring_buffer.enqueue(serial.line_buffer, false);
        parser.parse(serial.line_buffer);
          #if DEBUG_OUTPUT
            std::cout << "Line: " << serial.line_buffer << "\n";
            std::cout << "CMD: " << parser.command_letter << parser.codenum<< " ";   
            for (char ch = 'A'; ch <= 'Z'; ch++){
                if (parser.seen(ch)) {
                    std::cout << ch << ": " << parser.floatval(ch) << " ";
                }
            }
            std::cout << "\n";
        #endif
      }
      else
        process_stream_char(serial_char, serial.input_state, serial.line_buffer, serial.count);

    } // queue has space, serial has data
   
}

extern "C" int LLVMFuzzerTestOneInput(const uint8_t *Data, size_t Size){
    if (! SizeIsOk(Size)) {
        return 0;
    }

    simulate_serial_transmission(Data, Size);  
    
    return 0;
}
