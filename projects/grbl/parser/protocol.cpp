/*
  protocol.c - controls Grbl execution protocol and procedures
  Part of Grbl
  
  Copyright (c) 2011-2015 Sungeun K. Jeon  
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include "wrapper.h"

// Define different comment types for pre-parsing.
#define COMMENT_NONE 0
#define COMMENT_TYPE_PARENTHESES 1
#define COMMENT_TYPE_SEMICOLON 2


static char line[LINE_BUFFER_SIZE]; // Line to be executed. Zero-terminated.


// Directs and executes one line of formatted input from protocol_process. While mostly
// incoming streaming g-code blocks, this also directs and executes Grbl internal commands,
// such as settings, initiating the homing cycle, and toggling switch states.
static void protocol_execute_line(char *line) 
{      
  //protocol_execute_realtime(); // Runtime command check point.
  if (sys.abort) { return; } // Bail to calling function upon system abort  

  #ifdef REPORT_ECHO_LINE_RECEIVED
    report_echo_line_received(line);
  #endif

  if (line[0] == 0) {
    // Empty or comment line. Send status message for syncing purposes.
    return;

  } else if (line[0] == '$') {
    // Grbl '$' system command
    //report_status_message(system_execute_line(line));
    return;
    
  } else if (sys.state == STATE_ALARM) {
    // Everything else is gcode. Block if in alarm mode.
    //report_status_message(STATUS_ALARM_LOCK);
    return;

  } else {
    // Parse and execute g-code block!
    //report_status_message(gc_execute_line(line));
    gc_execute_line(line);
    #if DEBUG_OUTPUT
      print_debug();
    #endif
  }
}

#if DEBUG_OUTPUT
  void print_debug() {

    std::cout << "CMD: ";

    // print command
    if (bit_istrue(gc_block.command_words,bit(MODAL_GROUP_G0))) {
      std::cout << "[G4,G10,G28,G28.1,G30,G30.1,G53,G92,G92.1]";
    }
    if (bit_istrue(gc_block.command_words,bit(MODAL_GROUP_G1))) {
      std::cout << "[G0,G1,G2,G3,G38.2,G38.3,G38.4,G38.5,G80]";
    }
    if (bit_istrue(gc_block.command_words,bit(MODAL_GROUP_G2))) {
      std::cout << "[G17,G18,G19]";
    }
    if (bit_istrue(gc_block.command_words,bit(MODAL_GROUP_G3))) {
      std::cout << "[G90,G91]";
    }
    if (bit_istrue(gc_block.command_words,bit(MODAL_GROUP_G4))) {
      std::cout << "[G91.1]";
    }
    if (bit_istrue(gc_block.command_words,bit(MODAL_GROUP_G5))) {
      std::cout << "[G93,G94]";
    }
    if (bit_istrue(gc_block.command_words,bit(MODAL_GROUP_G6))) {
      std::cout << "[G20,G21]";
    }
    if (bit_istrue(gc_block.command_words,bit(MODAL_GROUP_G7))) {
      std::cout << "[G40]";
    }
    if (bit_istrue(gc_block.command_words,bit(MODAL_GROUP_G8))) {
      std::cout << "[G43.1,G49]";
    }
    if (bit_istrue(gc_block.command_words,bit(MODAL_GROUP_G12))) {
      std::cout << "[G54,G55,G56,G57,G58,G59]";
    }
    if (bit_istrue(gc_block.command_words,bit(MODAL_GROUP_G13))) {
      std::cout << "[G61]";
    }
    if (bit_istrue(gc_block.command_words,bit(MODAL_GROUP_M4))) {
      std::cout << "[M0,M1,M2,M30]";
    }
    if (bit_istrue(gc_block.command_words,bit(MODAL_GROUP_M7))) {
      std::cout << "[M3,M4,M5]";
    }
    if (bit_istrue(gc_block.command_words,bit(MODAL_GROUP_M8))) {
      std::cout << "[M7,M8,M9]";
    }


    // print values
    if (bit_istrue(gc_block.value_words,bit(WORD_X))) {
      std::cout << " X: " << gc_block.values.xyz[X_AXIS];
    }
    if (bit_istrue(gc_block.value_words,bit(WORD_Y))) {
      std::cout << " Y: " << gc_block.values.xyz[Y_AXIS];
    }
    if (bit_istrue(gc_block.value_words,bit(WORD_Z))) {
      std::cout << " Z: " << gc_block.values.xyz[Z_AXIS];
    }
    if (bit_istrue(gc_block.value_words,bit(WORD_F))) {
      std::cout << " F: " << gc_block.values.f;
    }
    if (bit_istrue(gc_block.value_words,bit(WORD_I))) {
      std::cout << " I: " << gc_block.values.ijk[X_AXIS];
    }
    if (bit_istrue(gc_block.value_words,bit(WORD_J))) {
      std::cout << " J: " << gc_block.values.ijk[Y_AXIS];
    }
    if (bit_istrue(gc_block.value_words,bit(WORD_K))) {
      std::cout << " K: " << gc_block.values.ijk[Z_AXIS];
    }
    if (bit_istrue(gc_block.value_words,bit(WORD_L))) {
      std::cout << " L: " << gc_block.values.l;
    }
    if (bit_istrue(gc_block.value_words,bit(WORD_N))) {
      std::cout << " N: " << gc_block.values.n;
    }
    if (bit_istrue(gc_block.value_words,bit(WORD_P))) {
      std::cout << " P: " << gc_block.values.p;
    }
    if (bit_istrue(gc_block.value_words,bit(WORD_R))) {
      std::cout << " R: " << gc_block.values.r;
    }
    if (bit_istrue(gc_block.value_words,bit(WORD_S))) {
      std::cout << " S: " << gc_block.values.s;
    }


    std::cout << "\n";
  }
#endif


/* 
  GRBL PRIMARY LOOP:
*/
void protocol_main_loop(const uint8_t* Data, size_t Size)
{

  sys.state = STATE_IDLE; // Set system to ready. Clear all state flags.

    
  // ---------------------------------------------------------------------------------  
  // Primary loop! Upon a system abort, this exits back to main() to reset the system. 
  // ---------------------------------------------------------------------------------  
  
  uint8_t comment = COMMENT_NONE;
  uint8_t char_counter = 0;
  uint8_t c;

  for (uint16_t index = 0; index < Size; index++) {

    // Process one line of incoming serial data, as the data becomes available. Performs an
    // initial filtering by removing spaces and comments and capitalizing all letters.
    
    // NOTE: While comment, spaces, and block delete(if supported) handling should technically 
    // be done in the g-code parser, doing it here helps compress the incoming data into Grbl's
    // line buffer, which is limited in size. The g-code standard actually states a line can't
    // exceed 256 characters, but the Arduino Uno does not have the memory space for this.
    // With a better processor, it would be very easy to pull this initial parsing out as a 
    // seperate task to be shared by the g-code parser and Grbl's system commands.
    
    c = Data[index];
      if ((c == '\n') || (c == '\r')) { // End of line reached
        line[char_counter] = 0; // Set string termination character.
        protocol_execute_line(line); // Line is complete. Execute it!
        comment = COMMENT_NONE;
        char_counter = 0;
      } else {
        if (comment != COMMENT_NONE) {
          // Throw away all comment characters
          if (c == ')') {
            // End of comment. Resume line. But, not if semicolon type comment.
            if (comment == COMMENT_TYPE_PARENTHESES) { comment = COMMENT_NONE; }
          }
        } else {
          if (c <= ' ') { 
            // Throw away whitepace and control characters  
          } else if (c == '/') { 
            // Block delete NOT SUPPORTED. Ignore character.
            // NOTE: If supported, would simply need to check the system if block delete is enabled.
          } else if (c == '(') {
            // Enable comments flag and ignore all characters until ')' or EOL.
            // NOTE: This doesn't follow the NIST definition exactly, but is good enough for now.
            // In the future, we could simply remove the items within the comments, but retain the
            // comment control characters, so that the g-code parser can error-check it.
            comment = COMMENT_TYPE_PARENTHESES;
          } else if (c == ';') {
            // NOTE: ';' comment to EOL is a LinuxCNC definition. Not NIST.
            comment = COMMENT_TYPE_SEMICOLON;
            
          // TODO: Install '%' feature 
          // } else if (c == '%') {
            // Program start-end percent sign NOT SUPPORTED.
            // NOTE: This maybe installed to tell Grbl when a program is running vs manual input,
            // where, during a program, the system auto-cycle start will continue to execute 
            // everything until the next '%' sign. This will help fix resuming issues with certain
            // functions that empty the planner buffer to execute its task on-time.

          } else if (char_counter >= (LINE_BUFFER_SIZE-1)) {
            // Detect line buffer overflow. Report error and reset line buffer.
            //report_status_message(STATUS_OVERFLOW);
            comment = COMMENT_NONE;
            char_counter = 0;
          } else if (c >= 'a' && c <= 'z') { // Upcase lowercase
            line[char_counter++] = c-'a'+'A';
          } else {
            line[char_counter++] = c;
          }
        
      }
    }
    
    // If there are no more characters in the serial read buffer to be processed and executed,
    // this indicates that g-code streaming has either filled the planner buffer or has 
    // completed. In either case, auto-cycle start, if enabled, any queued moves.
    //protocol_auto_cycle_start();

    //protocol_execute_realtime();  // Runtime command check point.
    if (sys.abort) { return; } // Bail to main() program loop to reset system.
              
  }
  
  return; /* Never reached */
}