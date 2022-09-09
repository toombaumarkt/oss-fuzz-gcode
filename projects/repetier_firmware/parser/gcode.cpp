/*
    This file is part of Repetier-Firmware.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.

    This firmware is a nearly complete rewrite of the sprinter firmware
    by kliment (https://github.com/kliment/Sprinter)
    which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm
  firmware.

  Functions in this file are used to communicate using ascii or repetier
  protocol.
*/


#include "wrapper.h"
#include "Communication.h"
#include "gcode.h"
#include <iostream>

#ifndef FEATURE_CHECKSUM_FORCED
#define FEATURE_CHECKSUM_FORCED false
#endif

GCode GCode::commandsBuffered[GCODE_BUFFER_SIZE];  ///< Buffer for received
                                                   ///< commands.
uint8_t GCode::bufferReadIndex = 0;                ///< Read position in gcode_buffer.
uint8_t GCode::bufferWriteIndex = 0;               ///< Write position in gcode_buffer.
uint8_t GCode::commandReceiving[MAX_CMD_SIZE];     ///< Current received command.
uint8_t GCode::commandsReceivingWritePosition = 0; ///< Writing position in gcode_transbuffer.
uint8_t GCode::sendAsBinary;                       ///< Flags the command as binary input.
uint8_t GCode::commentDetected = false;            ///< Flags true if we are reading the comment part of a command.
uint8_t
    GCode::binaryCommandSize;                      ///< Expected size of the incoming binary command.
bool GCode::waitUntilAllCommandsAreParsed = false; ///< Don't read until all commands are parsed. Needed if gcode_buffer
                                                   ///< is misused as storage for strings.
uint32_t GCode::actLineNumber;                     ///< Line number of current command.
volatile uint8_t GCode::bufferLength = 0;          ///< Number of commands stored in gcode_buffer
uint8_t GCode::formatErrors = 0;
const char* GCode::fatalErrorMsg = NULL;  ///< message unset = no fatal error
millis_t GCode::lastBusySignal = 0; ///< When was the last busy signal
uint32_t GCode::keepAliveInterval = KEEP_ALIVE_INTERVAL;
#if NEW_COMMUNICATION == 0
int8_t GCode::waitingForResend = -1;               ///< Waiting for line to be resend. -1 = no wait.
uint32_t GCode::lastLineNumber = 0;                ///< Last line number received.
uint8_t GCode::wasLastCommandReceivedAsBinary = 0; ///< Was the last successful command in binary mode?
millis_t GCode::timeOfLastDataPacket = 0;          ///< Time, when we got the last data packet. Used to detect missing
                                                   ///< uint8_ts.
#endif

/** \page Repetier-protocol

\section Introduction

The repetier-protocol was developed, to overcome some shortcomings in the
standard RepRap communication method, while remaining backward compatible. To
use the improved features of this protocol, you need a host which speaks it. On
Windows the recommended host software is Repetier-Host. It is developed in
parallel to this firmware and supports all implemented features.

\subsection Improvements

- With higher speeds, the serial connection is more likely to produce
communication failures. The standard method is to transfer a checksum at the end
of the line. This checksum is the XORd value of all characters send. The value
is limited to a range between 0 and 127. It can not detect two identical missing
characters or a wrong order. Therefore the new protocol uses Fletchers checksum,
which overcomes these shortcommings.
- The new protocol send data in binary format. This reduces the data size to
less then 50% and it speeds up decoding the command. No slow conversion from
string to floats are needed.

*/

/** \brief Computes size of binary data from bitfield.

In the repetier-protocol in binary mode, the first 2 uint8_ts define the
data. From this bitfield, this function computes the size of the command
including the 2 uint8_ts of the bitfield and the 2 uint8_ts for the checksum.

Gcode Letter to Bit and Datatype:

- N : Bit 0 : 16-Bit Integer
- M : Bit 1 :  8-Bit unsigned uint8_t
- G : Bit 2 :  8-Bit unsigned uint8_t
- X : Bit 3 :  32-Bit Float
- Y : Bit 4 :  32-Bit Float
- Z : Bit 5 :  32-Bit Float
- E : Bit 6 :  32-Bit Float
-  : Bit 7 :  always set to distinguish binary from ASCII line.
- F : Bit 8 :  32-Bit Float
- T : Bit 9 :  8 Bit Integer
- S : Bit 10 : 32 Bit Value
- P : Bit 11 : 32 Bit Integer
- V2 : Bit 12 : Version 2 command for additional commands/sizes
- Ext : Bit 13 : There are 2 more uint8_ts following with Bits, only for future
versions
- Int :Bit 14 : Marks it as internal command,
- Text : Bit 15 : 16 Byte ASCII String terminated with 0
Second word if V2:
- I : Bit 0 : 32-Bit float
- J : Bit 1 : 32-Bit float
- R : Bit 2 : 32-Bit float
- D : Bit 3 : 32-Bit float
- C : Bit 4 : 32-Bit float
- H : Bit 5 : 32-Bit float
- A : Bit 6 : 32-Bit float
- B : Bit 7 : 32-Bit float
- K : Bit 8 : 32-Bit float
- L : Bit 9 : 32-Bit float
- O : Bit 0 : 32-Bit float
*/

uint8_t GCode::computeBinarySize(char* ptr) // unsigned int bitfield) {
{
    uint8_t s = 4; // include checksum and bitfield
    uint16_t bitfield = *(uint16_t*)ptr;
    if (bitfield & 1)
        s += 2;
    #if FUZZ_TARGET_1_HARD_SANITY_CHECK
        if (bitfield & 8)
            s += 0;
    #else
        if (bitfield & 8)
            s += 4;
    #endif 
    if (bitfield & 16)
        s += 4;
    if (bitfield & 32)
        s += 4;
    if (bitfield & 64)
        s += 4;
    if (bitfield & 256)
        s += 4;
    if (bitfield & 512)
        s += 1;
        
    #if FUZZ_TARGET_1_HARD_SANITY_CHECK
        if (bitfield & 1024)
            s += 0;
        if (bitfield & 2048)
            s += 0;
    #else
        if (bitfield & 1024)
            s += 4;
        if (bitfield & 2048)
            s += 4;
    #endif
    
    if (bitfield & 4096) // Version 2 or later
    {
        s += 2; // for bitfield 2
        uint16_t bitfield2 = *(uint16_t*)(ptr + 2);
        if (bitfield & 2)
            s += 2;
        if (bitfield & 4)
            s += 2;
        if (bitfield2 & 1)
            s += 4;
        if (bitfield2 & 2)
            s += 4;
        if (bitfield2 & 4)
            s += 4;
        if (bitfield2 & 8)
            s += 4;
        if (bitfield2 & 16)
            s += 4;
        if (bitfield2 & 32)
            s += 4;
        if (bitfield2 & 64)
            s += 4;
        if (bitfield2 & 128)
            s += 4;
        if (bitfield2 & 256)
            s += 4;
        if (bitfield2 & 512)
            s += 4;
        if (bitfield2 & 1024)
            s += 4;
        if (bitfield2 & 2048)
            s += 4;
        if (bitfield2 & 4096)
            s += 4;
        if (bitfield2 & 8192)
            s += 4;
        if (bitfield2 & 16384)
            s += 4;
        if (bitfield2 & 32768)
            s += 4;
        if (bitfield & 32768)
            s += RMath::min(80, (uint8_t)ptr[4] + 1);
    } else {
        if (bitfield & 2)
            s += 1;
        if (bitfield & 4)
            s += 1;
        if (bitfield & 32768)
            s += 16;
    }
    return s;
}


/**
  Converts a binary uint8_tfield containing one GCode line into a GCode
  structure. Returns true if checksum was correct.
*/
bool GCode::parseBinary(uint8_t* buffer, fast8_t length, bool fromSerial) {
    internalCommand = !fromSerial;
    unsigned int sum1 = 0, sum2 = 0; // for fletcher-16 checksum
    // first do fletcher-16 checksum tests see
    // http://en.wikipedia.org/wiki/Fletcher's_checksum
    uint8_t* p = buffer;

    #if ENABLE_CHECKSUM_CALC
    uint8_t len = length - 2;
    while (len) {
        uint8_t tlen = len > 21 ? 21 : len;
        len -= tlen;
        do {
            sum1 += *p++;
            if (sum1 >= 255)
                sum1 -= 255;
            sum2 += sum1;
            if (sum2 >= 255)
                sum2 -= 255;
        } while (--tlen);
    }
    sum1 -= *p++;
    sum2 -= *p;
    if (sum1 | sum2) {
        // disabled
        /* if (Printer::debugErrors()) {
            Com::printErrorFLN(Com::tWrongChecksum);
        } */
        #if DEBUG_OUTPUT
            std::cout << "Checksum failed." << "\n";
        #endif
        return false;
    }
    #if DEBUG_OUTPUT
        std::cout << "Checksum succeded." << "\n";
    #endif
    #endif
    p = buffer;
    params = *(uint16_t*)p;
    #if FUZZ_TARGET_1_EASY_SANITY_CHECK
        p += 4;
    #else
        p += 2;
    #endif
    uint8_t textlen = 16;
    if (isV2()) {
        params2 = *(uint16_t*)p;
        p += 2;
        if (hasString())
        #if defined(FUZZING_BUILD_MODE_UNSAFE_FOR_PRODUCTION)
        // fixing bug related to line 376
            textlen = *p++;//RMath::min(80, *p++ + 1);
        #else
            textlen = *p++;
        #endif
    } else
        params2 = 0;
    if (params & 1) {
        actLineNumber = N = *(uint16_t*)p;
        p += 2;
    }
    if (isV2()) // Read G,M as 16 bit value
    {
        if (hasM()) {
            M = *(uint16_t*)p;
            p += 2;
        }
        if (hasG()) {
            G = *(uint16_t*)p;
            p += 2;
        }
    } else {
        if (hasM()) {
            M = *p++;
        }
        if (hasG()) {
            G = *p++;
        }
    }
    // if(code->params & 8) {memcpy(&code->X,p,4);p+=4;}
    if (hasX()) {
        X = *(float*)p;
        p += 4;
    }
    if (hasY()) {
        Y = *(float*)p;
        p += 4;
    }
    if (hasZ()) {
        Z = *(float*)p;
        p += 4;
    }
    if (hasE()) {
        E = *(float*)p;
        p += 4;
    }
    if (hasF()) {
        F = *(float*)p;
        p += 4;
    }
    if (hasT()) {
        T = *p++;
    }
    if (hasS()) {
        S = *(int32_t*)p;
        p += 4;
    }
    if (hasP()) {
        P = *(int32_t*)p;
        p += 4;
    }
    if (hasI()) {
        I = *(float*)p;
        p += 4;
    }
    if (hasJ()) {
        J = *(float*)p;
        p += 4;
    }
    if (hasR()) {
        R = *(float*)p;
        p += 4;
    }
    if (hasD()) {
        D = *(float*)p;
        p += 4;
    }
    if (hasC()) {
        C = *(float*)p;
        p += 4;
    }
    if (hasH()) {
        H = *(float*)p;
        p += 4;
    }
    if (hasA()) {
        A = *(float*)p;
        p += 4;
    }
    if (hasB()) {
        B = *(float*)p;
        p += 4;
    }
    if (hasK()) {
        K = *(float*)p;
        p += 4;
    }
    if (hasL()) {
        L = *(float*)p;
        p += 4;
    }
    if (hasO()) {
        O = *(float*)p;
        p += 4;
    }
    if (hasString()) // set text pointer to string
    {
        text = (char*)p;
        text[textlen] = 0;                    // Terminate string overwriting checksum
        waitUntilAllCommandsAreParsed = true; // Don't destroy string until executed
    }
    formatErrors = 0;
    return true;
}

#if DEBUG_OUTPUT
/** \brief Print command on serial console */
void GCode::printCommand() {
    if (hasN()) {
        std::cout << "N: " << (int32_t)N << " ";
    }
    if (hasM()) {
        std::cout << "M: " << (int)M << " ";
    }
    if (hasG()) {
        std::cout << "G: " << (int)G << " ";
    }
    if (hasT()) {
        std::cout << "T: " << (int)T << " ";
    }
    if (hasX()) {
        std::cout << "X: " << X << " ";
    }
    if (hasY()) {
        std::cout << "Y: " << Y << " ";
    }
    if (hasZ()) {
        std::cout << "Z: " << Z << " ";
    }
    if (hasE()) {
        std::cout << "E: " << E << " ";
    }
    if (hasF()) {
        std::cout << "F: " << F << " ";
    }
    if (hasS()) {
        std::cout << "S: " << S << " ";
    }
    if (hasP()) {
        std::cout << "P: " << P << " ";
    }
    if (hasI()) {
        std::cout << "I: " << I << " ";
    }
    if (hasJ()) {
        std::cout << "J: " << J << " ";
    }
    if (hasR()) {
        std::cout << "R: " << R << " ";
    }
    if (hasD()) {
        std::cout << "D: " << D << " ";
    }
    if (hasC()) {
        std::cout << "C: " << C << " ";
    }
    if (hasH()) {
        std::cout << "H: " << H << " ";
    }
    if (hasA()) {
        std::cout << "A: " << A << " ";
    }
    if (hasB()) {
        std::cout << "B: " << B << " ";
    }
    if (hasK()) {
        std::cout << "K: " << K << " ";
    }
    if (hasL()) {
        std::cout << "L: " << L << " ";
    }
    if (hasO()) {
        std::cout << "O: " << O << " ";
    }
    if (hasString()) {
        std::cout << "text: " << text << " ";
    }
     std::cout << "\n";
}
#endif

void GCode::requestResend() {
    commandsReceivingWritePosition = 0;
#if NEW_COMMUNICATION
    if (sendAsBinary)
        GCodeSource::activeSource->waitingForResend = 30;
    else
        GCodeSource::activeSource->waitingForResend = 14;
    //Com::println();
    //Com::printFLN(Com::tResend, GCodeSource::activeSource->lastLineNumber + 1);
#else
    if (sendAsBinary)
        waitingForResend = 30;
    else
        waitingForResend = 14;
    Com::println();
    Com::printFLN(Com::tResend, lastLineNumber + 1);
#endif
    //Com::printFLN(Com::tOk);
}

// inserted
void GCode::reset(){
    bufferReadIndex = 0;                ///< Read position in gcode_buffer.
    bufferWriteIndex = 0;               ///< Write position in gcode_buffer.
    commandReceiving[MAX_CMD_SIZE - 1] = {};     ///< Current received command.
    commandsReceivingWritePosition = 0; ///< Writing position in gcode_transbuffer.
    sendAsBinary = 0;                       ///< Flags the command as binary input.
    commentDetected = false;            ///< Flags true if we are reading the comment part of a command.
    binaryCommandSize = 0;                      ///< Expected size of the incoming binary command.
    waitUntilAllCommandsAreParsed = false; ///< Don't read until all commands are parsed. Needed if gcode_buffer
                                                   ///< is misused as storage for strings.
    actLineNumber = 0;                     ///< Line number of current command.
    bufferLength = 0;          ///< Number of commands stored in gcode_buffer
    formatErrors = 0;
    fatalErrorMsg = NULL;  ///< message unset = no fatal error
    lastBusySignal = 0; ///< When was the last busy signal
    keepAliveInterval = KEEP_ALIVE_INTERVAL;
}

void GCode::keepAlive(enum FirmwareState state) {
    //millis_t now = HAL::timeInMilliseconds();

    if (state != NotBusy && keepAliveInterval != 0) {
        if (/*now -*/ lastBusySignal < keepAliveInterval)
            return;
        if (state == Paused) {
            //GCodeSource::printAllFLN(PSTR("busy:paused for user interaction"));
        } else if (state == WaitHeater) {
            //GCodeSource::printAllFLN(PSTR("busy:heating"));
        } else if (state == DoorOpen) {
            //GCodeSource::printAllFLN(PSTR("busy:door open"));
            //UI_STATUS_F(Com::tDoorOpen);
        } else { // processing and uncaught cases
            //GCodeSource::printAllFLN(PSTR("busy:processing"));
        }
    }
    //lastBusySignal = now;
}

void GCode::checkAndPushCommand() {
    if (hasM()) {
        if (M == 110) // Reset line number
        {
#if NEW_COMMUNICATION
            GCodeSource::activeSource->lastLineNumber = actLineNumber;
            //Com::printFLN(Com::tOk);
            GCodeSource::activeSource->waitingForResend = -1;
#else
            lastLineNumber = actLineNumber;
            Com::printFLN(Com::tOk);
            waitingForResend = -1;
#endif
            return;
        }
        if (M == 112) // Emergency kill - freeze printer
        {
            //Commands::emergencyStop();
        }
#ifdef DEBUG_COM_ERRORS
        if (M == 666) // force an communication error
        {
#if NEW_COMMUNICATION
            GCodeSource::activeSource->lastLineNumber++;
#else
            lastLineNumber++;
#endif
            return;
        } else if (M == 668) {
#if NEW_COMMUNICATION
            GCodeSource::activeSource->lastLineNumber = 0; // simulate a reset so lines are out of resend buffer
#else
            lastLineNumber = 0; // simulate a reset so lines are out of resend buffer
#endif
        }
#endif // DEBUG_COM_ERRORS
    }
    if (hasN()) {
#if NEW_COMMUNICATION
        if ((((GCodeSource::activeSource->lastLineNumber + 1) & 0xffff) != (actLineNumber & 0xffff)))
#else
        if ((((lastLineNumber + 1) & 0xffff) != (actLineNumber & 0xffff)))
#endif
        {
#if NEW_COMMUNICATION
            if (static_cast<uint16_t>(GCodeSource::activeSource->lastLineNumber - actLineNumber) < 40)
#else
            if (static_cast<uint16_t>(lastLineNumber - actLineNumber) < 40)
#endif
            {
                // we have seen that line already. So we assume it is a repeated resend
                // and we ignore it
                commandsReceivingWritePosition = 0;
                //Com::printFLN(Com::tSkip, actLineNumber);
                //Com::printFLN(Com::tOk);
            }
#if NEW_COMMUNICATION
            else if (GCodeSource::activeSource->waitingForResend < 0) // after a resend, we have to skip the garbage in buffers, no
                                                                      // message for this
#else
            else if (waitingForResend < 0) // after a resend, we have to skip the
                                           // garbage in buffers, no message for this
#endif
            {
                /*if (Printer::debugErrors()) {
#if NEW_COMMUNICATION
                    Com::printF(Com::tExpectedLine,
                                GCodeSource::activeSource->lastLineNumber + 1);
#else
                    Com::printF(Com::tExpectedLine, lastLineNumber + 1);
#endif
                    Com::printFLN(Com::tGot, actLineNumber);
                }*/
                requestResend(); // Line missing, force resend
            } else {
#if NEW_COMMUNICATION
                --GCodeSource::activeSource->waitingForResend;
#else
                --waitingForResend;
#endif
                commandsReceivingWritePosition = 0;
                //Com::printFLN(Com::tSkip, actLineNumber);
                //Com::printFLN(Com::tOk);
            }
            return;
        }
#if NEW_COMMUNICATION
        GCodeSource::activeSource->lastLineNumber = actLineNumber;
#else
        lastLineNumber = actLineNumber;
#endif
    } /*
      This test is not compatible with all hosts. Replaced by forbidding
      backward switch of protocols. else if(lastLineNumber && !(hasM() && M ==
      117)) { // once line number always line number! if(Printer::debugErrors())
      {
                      Com::printErrorFLN(PSTR("Missing linenumber"));
              }
              requestResend();
              return;
      }*/
    if (GCode::hasFatalError() && !(hasM() && M == 999)) {
        //GCode::reportFatalError();
    } else {
        pushCommand();
    }
#ifdef DEBUG_COM_ERRORS
    if (hasM() && M == 667)
        return; // omit ok
#endif
#if ACK_WITH_LINENUMBER
    Com::printFLN(Com::tOkSpace, actLineNumber);
#else
    //Com::printFLN(Com::tOk);
#endif
#if NEW_COMMUNICATION
    GCodeSource::activeSource->wasLastCommandReceivedAsBinary = sendAsBinary;
    keepAlive(NotBusy);
    GCodeSource::activeSource->waitingForResend = -1; // everything is ok.
#else
    wasLastCommandReceivedAsBinary = sendAsBinary;
    keepAlive(NotBusy);
    waitingForResend = -1; // everything is ok.
#endif
}


void GCode::pushCommand() {
#if !ECHO_ON_EXECUTE
    commandsBuffered[bufferWriteIndex].echoCommand();
#endif
    if (++bufferWriteIndex >= GCODE_BUFFER_SIZE)
        bufferWriteIndex = 0;
    bufferLength++;
}

void GCode::readFromSerial() {
#if defined(DOOR_PIN) && DOOR_PIN > -1
    if (Printer::isDoorOpen()) {
        keepAlive(DoorOpen);
        return; // do nothing while door is open
    }
#endif
#if EMERGENCY_PARSER
    GCodeSource::prefetchAll();
#endif
    if (bufferLength >= GCODE_BUFFER_SIZE || (waitUntilAllCommandsAreParsed && bufferLength)) {
        keepAlive(Processing);
        return; // all buffers full
    }
    waitUntilAllCommandsAreParsed = false;
    //millis_t time = HAL::timeInMilliseconds();

#if NEW_COMMUNICATION
    //bool lastWTA = Com::writeToAll;
    //Com::writeToAll = false;
    if (!GCodeSource::activeSource->dataAvailable()) {
        if (GCodeSource::activeSource
                ->closeOnError()) {                   // this device does not support resends so all
                                                      // errors are final and we always expect there
                                                      // is a new char!
            if (commandsReceivingWritePosition > 0) { // it's only an error if we have started reading a command
                GCodeSource::activeSource->close();
                //GCodeSource::rotateSource();
                //Com::writeToAll = lastWTA;
                return;
            }
        } else {
            if ((GCodeSource::activeSource->waitingForResend >= 0 || commandsReceivingWritePosition > 0) /*&& time - GCodeSource::activeSource->timeOfLastDataPacket > 200*/) // only if we get no further data after 200ms it is a problem
            {
                // Com::printF(PSTR("WFR:"),waitingForResend);Com::printF(PSTR("
                // CRWP:"),commandsReceivingWritePosition);commandReceiving[commandsReceivingWritePosition]
                // = 0;Com::printFLN(PSTR(" GOT:"),(char*)commandReceiving);
                requestResend(); // Something is wrong, a started line was not continued
                                 // in the last second
                //GCodeSource::activeSource->timeOfLastDataPacket = time;
            }
#ifdef WAITING_IDENTIFIER
            else if (bufferLength == 0 && time - GCodeSource::activeSource->timeOfLastDataPacket > 1000) // Don't do it if buffer is not empty. It may be a slow
                                                                                                         // executing command.
            {
                Com::printFLN(Com::tWait); // Unblock communication in case the last ok
                                           // was not received correct.
                GCodeSource::activeSource->timeOfLastDataPacket = time;
            }
#endif
        }
        //if (commandsReceivingWritePosition == 0) // nothing read, we can rotate to next input source
            //GCodeSource::rotateSource();
    }
    while (GCodeSource::activeSource->dataAvailable() && commandsReceivingWritePosition < MAX_CMD_SIZE) // consume data until no data or buffer full
    {
        //GCodeSource::activeSource->timeOfLastDataPacket = time; // HAL::timeInMilliseconds();
        commandReceiving[commandsReceivingWritePosition++] = GCodeSource::activeSource->readByte();
        
        #if DEBUG_OUTPUT
            std::cout << "Received: " << std::hex << unsigned(commandReceiving[commandsReceivingWritePosition - 1]) << "\n";
        #endif

        // first lets detect, if we got an old type ascii command
        if (commandsReceivingWritePosition == 1 && commentDetected == false) {
            if (GCodeSource::activeSource->waitingForResend >= 0 && GCodeSource::activeSource->wasLastCommandReceivedAsBinary) {
                if (!commandReceiving[0])
                    GCodeSource::activeSource
                        ->waitingForResend--; // Skip 30 zeros to get in sync
                else
                    GCodeSource::activeSource->waitingForResend = 30;
                commandsReceivingWritePosition = 0;
                continue;
            }
            if (!commandReceiving[0]) // Ignore zeros
            {
                commandsReceivingWritePosition = 0;
                //GCodeSource::rotateSource(); // could also be end of file, so let's
                                             // rotate source if it closed it self
                //Com::writeToAll = lastWTA;
                return;
            }
            // Inserted --> Select Binary or Ascii parsing
            /*
            #if FUZZ_TARGET == 0
                sendAsBinary = 0;
            #else
                sendAsBinary = 1;
            #endif
            */
            // disabled
            sendAsBinary = (commandReceiving[0] & 128) != 0;
            #if DEBUG_OUTPUT
                std::cout << std::hex << unsigned(sendAsBinary) << "\n";
            #endif
        } // first byte detection
        if (sendAsBinary) {
            if (commandsReceivingWritePosition < 2)
                continue;
            if (commandsReceivingWritePosition == 5 || commandsReceivingWritePosition == 4){
                binaryCommandSize = computeBinarySize((char*)commandReceiving);
            }
            #if DEBUG_OUTPUT
                    std::cout << "Size: " << unsigned(binaryCommandSize) << " WritePosition:  " << unsigned(commandsReceivingWritePosition) << "\n";
                #endif
            if (commandsReceivingWritePosition == binaryCommandSize) {
                GCode* act = &commandsBuffered[bufferWriteIndex];
                #if DEBUG_OUTPUT
                    commandReceiving[95] = '\0';
                    std::cout << "Received Binary: " << commandReceiving << "\n";
                #endif
                act->source = GCodeSource::activeSource; // we need to know where to
                                                         // write answers to
                if (act->parseBinary(commandReceiving, binaryCommandSize,
                                     true)) { // Success
                    act->checkAndPushCommand();
                    #if DEBUG_OUTPUT
                        act->printCommand();
                    #endif
                } else {
                    if (GCodeSource::activeSource
                            ->closeOnError()) { // this device does not support resends so
                                                // all errors are final!
                        GCodeSource::activeSource->close();
                    } else {
                        requestResend();
                    }
                }
                // disabled
                //GCodeSource::rotateSource();
                //Com::writeToAll = lastWTA;
                return;
            }
        } else // ASCII command
        {
            char ch = commandReceiving[commandsReceivingWritePosition - 1];
            if (ch == 0 || ch == '\n' || ch == '\r' || !GCodeSource::activeSource->isOpen() /*|| (!commentDetected && ch == ':')*/) // complete
                                                                                                                                    // line read
            {
                commandReceiving[commandsReceivingWritePosition - 1] = 0;
#ifdef DEBUG_ECHO_ASCII
                Com::printF(PSTR("Got:"));
                Com::print((char*)commandReceiving);
                Com::println();
#endif
                #if DEBUG_OUTPUT
                    commandReceiving[95] = '\0';
                    std::cout << "Received ASCII: " << commandReceiving << "\n";
                #endif
                commentDetected = false;
                if (commandsReceivingWritePosition == 1) // empty line ignore
                {
                    commandsReceivingWritePosition = 0;
                    continue;
                }
                GCode* act = &commandsBuffered[bufferWriteIndex];
                act->source = GCodeSource::activeSource;              // we need to know where to
                                                                      // write answers to
                if (act->parseAscii((char*)commandReceiving, true)) { // Success
                    act->checkAndPushCommand();
                    #if DEBUG_OUTPUT
                        act->printCommand();
                    #endif
                } else {
                    if (GCodeSource::activeSource
                            ->closeOnError()) { // this device doe snot support resends so
                                                // all errors are final!
                        GCodeSource::activeSource->close();
                    } else {
                        requestResend();
                    }
                }
                //GCodeSource::rotateSource();
                commandsReceivingWritePosition = 0;
                //Com::writeToAll = lastWTA;
                return;
            } else {
                if (ch == ';')
                    commentDetected = true; // ignore new data until line end
                if (commentDetected)
                    commandsReceivingWritePosition--;
            }
        }
        if (commandsReceivingWritePosition == MAX_CMD_SIZE) {
            if (GCodeSource::activeSource
                    ->closeOnError()) { // this device does not support resends so all
                                        // errors are final!
                GCodeSource::activeSource->close();
                //GCodeSource::rotateSource();
            } else {
                requestResend();
            }
        }
    } // while
    //Com::writeToAll = lastWTA;
#else
    if (!HAL::serialByteAvailable()) {
        if ((waitingForResend >= 0 || commandsReceivingWritePosition > 0) && time - timeOfLastDataPacket > 200) {
            // Com::printF(PSTR("WFR:"),waitingForResend);Com::printF(PSTR("
            // CRWP:"),commandsReceivingWritePosition);commandReceiving[commandsReceivingWritePosition]
            // = 0;Com::printFLN(PSTR(" GOT:"),(char*)commandReceiving);
            requestResend(); // Something is wrong, a started line was not continued
                             // in the last second
            timeOfLastDataPacket = time;
        }
#ifdef WAITING_IDENTIFIER
        else if (bufferLength == 0 && time - timeOfLastDataPacket > 1000) // Don't do it if buffer is not empty. It may be a slow
                                                                          // executing command.
        {
            Com::printFLN(Com::tWait); // Unblock communication in case the last ok
                                       // was not received correct.
            timeOfLastDataPacket = time;
        }
#endif
    }
    while (HAL::serialByteAvailable() && commandsReceivingWritePosition < MAX_CMD_SIZE) // consume data until no data or buffer full
    {
        timeOfLastDataPacket = time; // HAL::timeInMilliseconds();
        commandReceiving[commandsReceivingWritePosition++] = HAL::serialReadByte();
        // first lets detect, if we got an old type ascii command
        if (commandsReceivingWritePosition == 1) {
            if (waitingForResend >= 0 && wasLastCommandReceivedAsBinary) {
                if (!commandReceiving[0])
                    waitingForResend--; // Skip 30 zeros to get in sync
                else
                    waitingForResend = 30;
                commandsReceivingWritePosition = 0;
                continue;
            }
            if (!commandReceiving[0]) // Ignore zeros
            {
                commandsReceivingWritePosition = 0;
                continue;
            }
            sendAsBinary = (commandReceiving[0] & 128) != 0;
        }
        if (sendAsBinary) {
            if (commandsReceivingWritePosition < 2)
                continue;
            if (commandsReceivingWritePosition == 5 || commandsReceivingWritePosition == 4)
                binaryCommandSize = computeBinarySize((char*)commandReceiving);
            if (commandsReceivingWritePosition == binaryCommandSize) {
                GCode* act = &commandsBuffered[bufferWriteIndex];
                if (act->parseBinary(commandReceiving, binaryCommandSize,
                                     true)) // Success
                    act->checkAndPushCommand();
                else
                    requestResend();
                commandsReceivingWritePosition = 0;
                return;
            }
        } else // ASCII command
        {
            char ch = commandReceiving[commandsReceivingWritePosition - 1];
            if (ch == 0 || ch == '\n' || ch == '\r' /*|| (!commentDetected && ch == ':')*/) // complete line read
            {
                commandReceiving[commandsReceivingWritePosition - 1] = 0;
#ifdef DEBUG_ECHO_ASCII
                Com::printF(PSTR("Got:"));
                Com::print((char*)commandReceiving);
                Com::println();
#endif
                commentDetected = false;
                if (commandsReceivingWritePosition == 1) // empty line ignore
                {
                    commandsReceivingWritePosition = 0;
                    continue;
                }
                GCode* act = &commandsBuffered[bufferWriteIndex];
                if (act->parseAscii((char*)commandReceiving, true)) // Success
                    act->checkAndPushCommand();
                else
                    requestResend();
                commandsReceivingWritePosition = 0;
                return;
            } else {
                if (ch == ';')
                    commentDetected = true; // ignore new data until line end
                if (commentDetected)
                    commandsReceivingWritePosition--;
            }
        }
        if (commandsReceivingWritePosition == MAX_CMD_SIZE) {
            requestResend();
            return;
        }
    }
#if SDSUPPORT
    if (sd.sdmode == 0 || sd.sdmode >= 100 || commandsReceivingWritePosition != 0) // not reading or incoming serial command
        return;
    while (sd.filesize > sd.sdpos && commandsReceivingWritePosition < MAX_CMD_SIZE) // consume data until no data or buffer full
    {
        timeOfLastDataPacket = HAL::timeInMilliseconds();
        int n = sd.file.read();
        if (n == -1) {
            Com::printFLN(Com::tSDReadError);
            UI_ERROR("SD Read Error");

            // Second try in case of recoverable errors
            sd.file.seekSet(sd.sdpos);
            n = sd.file.read();
            if (n == -1) {
                Com::printErrorFLN(PSTR("SD error did not recover!"));
                sd.sdmode = 0;
                break;
            }
            UI_ERROR("SD error fixed");
        }
        sd.sdpos++; // = file.curPosition();
        commandReceiving[commandsReceivingWritePosition++] = (uint8_t)n;

        // first lets detect, if we got an old type ascii command
        if (commandsReceivingWritePosition == 1) {
            sendAsBinary = (commandReceiving[0] & 128) != 0;
        }
        if (sendAsBinary) {
            if (commandsReceivingWritePosition < 2)
                continue;
            if (commandsReceivingWritePosition == 4 || commandsReceivingWritePosition == 5)
                binaryCommandSize = computeBinarySize((char*)commandReceiving);
            if (commandsReceivingWritePosition == binaryCommandSize) {
                GCode* act = &commandsBuffered[bufferWriteIndex];
                if (act->parseBinary(
                        commandReceiving,
                        false)) // Success, silently ignore illegal commands
                    pushCommand();
                commandsReceivingWritePosition = 0;
                if (sd.sdmode == 2)
                    sd.sdmode = 0;
                return;
            }
        } else {
            char ch = commandReceiving[commandsReceivingWritePosition - 1];
            bool returnChar = ch == '\n' || ch == '\r';
            if (returnChar || sd.filesize == sd.sdpos || (!commentDetected && ch == ':') || commandsReceivingWritePosition >= (MAX_CMD_SIZE - 1)) // complete line read
            {
                if (returnChar || ch == ':')
                    commandReceiving[commandsReceivingWritePosition - 1] = 0;
                else
                    commandReceiving[commandsReceivingWritePosition] = 0;
                commentDetected = false;
                if (commandsReceivingWritePosition == 1) // empty line ignore
                {
                    commandsReceivingWritePosition = 0;
                    continue;
                }
                GCode* act = &commandsBuffered[bufferWriteIndex];
                if (act->parseAscii((char*)commandReceiving, false)) // Success
                    pushCommand();
                commandsReceivingWritePosition = 0;
                if (sd.sdmode == 2)
                    sd.sdmode = 0;
                return;
            } else {
                if (ch == ';')
                    commentDetected = true; // ignore new data until line end
                if (commentDetected)
                    commandsReceivingWritePosition--;
            }
        }
    }
    sd.sdmode = 0;
    Com::printFLN(Com::tDonePrinting);
    commandsReceivingWritePosition = 0;
    commentDetected = false;
    Printer::setMenuMode(MENU_MODE_SD_PRINTING, false);
#endif
#endif
}

/** \brief Removes the last returned command from cache. */
void GCode::popCurrentCommand() {
    if (!bufferLength)
        return; // Should not happen, but safety first
#if ECHO_ON_EXECUTE
    //echoCommand();
#endif
    if (++bufferReadIndex == GCODE_BUFFER_SIZE)
        bufferReadIndex = 0;
    bufferLength--;
}
/**
  Converts a ASCII GCode line into a GCode structure.
*/
bool GCode::parseAscii(char* line, bool fromSerial) {
    char* pos = line;
    params = 0;
    params2 = 0;
    internalCommand = !fromSerial;
    bool hasChecksum = false;
    char c;
    while ((c = *(pos++))) {
        if (c == '(' || c == '%')
            break; // alternative comment or program block
        switch (c) {
        case 'N':
        case 'n': {
            actLineNumber = parseLongValue(pos);
            params |= 1;
            N = actLineNumber;
            break;
        }
        case 'G':
        case 'g': {
            G = parseLongValue(pos) & 0xffff;
            params |= 4;
            if (G > 255)
                params |= 4096;
            break;
        }
        case 'M':
        case 'm': {
            M = parseLongValue(pos) & 0xffff;
            params |= 2;
            if (M > 255)
                params |= 4096;
            // handle non standard text arguments that some M codes have
            if (M == 20 || M == 23 || M == 28 || M == 29 || M == 30 || M == 32 || M == 36 || M == 117 || M == 118 || M == 531) {
                // after M command we got a filename or text
                char digit;
                while ((digit = *pos)) {
                    if (digit < '0' || digit > '9')
                        break;
                    pos++;
                }
                while ((digit = *pos)) {
                    if (digit != ' ')
                        break;
                    pos++;
                    // skip leading white spaces (may be no white space)
                }
                text = pos;
                while (*pos) {
                    if ((M != 117 && M != 20 && M != 531 && *pos == ' ') || *pos == '*')
                        break;
                    pos++; // find a space as file name end
                }
                *pos = 0;                             // truncate filename by erasing space with null, also skips
                                                      // checksum
                waitUntilAllCommandsAreParsed = true; // don't risk string be deleted
                params |= 32768;
            }
            break;
        }
        case 'X':
        case 'x': {
            X = parseFloatValue(pos);
            params |= 8;
            break;
        }
        case 'Y':
        case 'y': {
            Y = parseFloatValue(pos);
            params |= 16;
            break;
        }
        case 'Z':
        case 'z': {
            Z = parseFloatValue(pos);
            params |= 32;
            break;
        }
        case 'E':
        case 'e': {
            E = parseFloatValue(pos);
            params |= 64;
            break;
        }
        case 'F':
        case 'f': {
            F = parseFloatValue(pos);
            params |= 256;
            break;
        }
        case 'T':
        case 't': {
            T = parseLongValue(pos) & 0xff;
            params |= 512;
            break;
        }
        case 'S':
        case 's': {
            S = parseLongValue(pos);
            params |= 1024;
            break;
        }
        case 'P':
        case 'p': {
            P = parseLongValue(pos);
            params |= 2048;
            break;
        }
        case 'I':
        case 'i': {
            I = parseFloatValue(pos);
            params2 |= 1;
            params |= 4096; // Needs V2 for saving
            break;
        }
        case 'J':
        case 'j': {
            J = parseFloatValue(pos);
            params2 |= 2;
            params |= 4096; // Needs V2 for saving
            break;
        }
        case 'R':
        case 'r': {
            R = parseFloatValue(pos);
            params2 |= 4;
            params |= 4096; // Needs V2 for saving
            break;
        }
        case 'D':
        case 'd': {
            D = parseFloatValue(pos);
            params2 |= 8;
            params |= 4096; // Needs V2 for saving
            break;
        }
        case 'C':
        case 'c': {
            C = parseFloatValue(pos);
            params2 |= 16;
            params |= 4096; // Needs V2 for saving
            break;
        }
        case 'H':
        case 'h': {
            H = parseFloatValue(pos);
            params2 |= 32;
            params |= 4096; // Needs V2 for saving
            break;
        }
        case 'A':
        case 'a': {
            A = parseFloatValue(pos);
            params2 |= 64;
            params |= 4096; // Needs V2 for saving
            break;
        }
        case 'B':
        case 'b': {
            B = parseFloatValue(pos);
            params2 |= 128;
            params |= 4096; // Needs V2 for saving
            break;
        }
        case 'K':
        case 'k': {
            K = parseFloatValue(pos);
            params2 |= 256;
            params |= 4096; // Needs V2 for saving
            break;
        }
        case 'L':
        case 'l': {
            L = parseFloatValue(pos);
            params2 |= 512;
            params |= 4096; // Needs V2 for saving
            break;
        }
        case 'O':
        case 'o': {
            O = parseFloatValue(pos);
            params2 |= 1024;
            params |= 4096; // Needs V2 for saving
            break;
        }
        case '*': // checksum
        {
            uint8_t checksum_given = parseLongValue(pos);
            uint8_t checksum = 0;
            while (line != (pos - 1))
                checksum ^= *line++;
#if FEATURE_CHECKSUM_FORCED
            Printer::flag0 |= PRINTER_FLAG0_FORCE_CHECKSUM;
#endif
            #if not defined(FUZZING_BUILD_MODE_UNSAFE_FOR_PRODUCTION)              
                if (checksum != checksum_given) {
                    return false; // mismatch
                }
            #endif
            hasChecksum = true;
            break;
        }
        default:
            break;
        } // end switch
    }     // end while
#if NEW_COMMUNICATION
    if (GCodeSource::activeSource->wasLastCommandReceivedAsBinary && !hasChecksum && fromSerial && !waitUntilAllCommandsAreParsed) {
#else
    if (wasLastCommandReceivedAsBinary && !hasChecksum && fromSerial && !waitUntilAllCommandsAreParsed) {
#endif
        return false;
    }
    if (hasFormatError() /*|| (params & 518) == 0*/) // Must contain G, M or T
                                                     // command and parameter need
                                                     // to have variables!
    {
        formatErrors++;
        if (formatErrors < 3)
            return false;
    } else
        formatErrors = 0;
    return true;
}

GCodeSource* GCodeSource::activeSource;

GCodeSource::GCodeSource() {
    lastLineNumber = 0;
    wasLastCommandReceivedAsBinary = false;
    waitingForResend = -1;
}

FuzzingGCodeSource::FuzzingGCodeSource(const uint8_t * Data, size_t Size) : GCodeSource() { 
    this->Data = Data;
    this->Size = Size;
    this->BytesRead = 0;
    }
bool FuzzingGCodeSource::isOpen() { return Size > 0 && BytesRead < Size; }
bool FuzzingGCodeSource::supportsWrite() { ///< true if write is a non dummy
                                         ///< function
    return false;
}
bool FuzzingGCodeSource::closeOnError() { // return true if the channel can not
                                        // interactively correct errors.
    return false;
}
bool FuzzingGCodeSource::dataAvailable() { // would read return a new byte?
    return BytesRead < Size;
}
int FuzzingGCodeSource::readByte() {
    if (BytesRead >= Size) {
        return -1;
    }
    uint8_t byte;
    byte = this->Data[this->BytesRead++];
    #if DEBUG_OUTPUT
        std::cout << "Read Byte: " << std::hex << unsigned(byte) << "\n";
    #endif
    return byte;
}
void FuzzingGCodeSource::close() {
    //dummy
}

void FuzzingGCodeSource::writeByte(uint8_t byte) {
    // dummy
}
