#include <iostream>
#include <fstream>
#include <string.h>
#include <stdio.h>
#include <sstream>
#include <vector>

#include "wrapper.h"


std::stringstream ConvertByteDataToStringStream(const uint8_t *Data, size_t Size) {
    char* fuzzed_input = new char[Size + 1];
    memcpy(fuzzed_input, Data, Size);
    fuzzed_input[Size] = '\0';
    std::stringstream ss(fuzzed_input);
    delete[]fuzzed_input;
    return ss;
}

bool SizeIsOk(size_t Size) {
    return Size >= MIN_DATA_SIZE && Size <= MAX_DATA_SIZE;
}

/****** OBSOLETE *******/
/* extern "C" int LLVMFuzzerTestOneInput(const uint8_t *Data, size_t Size){
    if (! SizeIsOk(Size)) {
        return -1;
    }

    // dummy GCodeSource to prevent nullexception at gcode.cpp line 1156
    GCodeSource::activeSource = new DummyGCodeSource();
    
    // convert bytes to string so it cant be easily split into lines
    std::stringstream ss = ConvertByteDataToStringStream(Data, Size);
    std::string line;
    
    // main loop
    GCode* act = new GCode();
    while (std::getline(ss, line)){
        // copying gcode_line into chararray
        char* gcode_line = new char[line.length() + 1];
        strcpy(gcode_line, line.c_str());
        #if DEBUG_OUTPUT
            std::cout << "Received: " << gcode_line << "\n";
        #endif
        // act
        act->parseAscii(gcode_line, false);

        #if DEBUG_OUTPUT
            act->printCommand();
        #endif

        // clean up
        delete[] gcode_line;
    }

    delete(act);
    delete(GCodeSource::activeSource);
    return 0;
}


extern "C" int LLVMFuzzerTestOneInput(const uint8_t *Data, size_t Size){
    if (Size < 5) {
        return -1;
    }

    // dummy GCodeSource to prevent nullexception at gcode.cpp line 1156
    GCodeSource::activeSource = new DummyGCodeSource();

    // convert bytes to string so it cant be easily split into lines
    // std::stringstream ss = ConvertByteDataToStringStream(Data, Size);
    // std::string line;
    uint8_t* gcode = new uint8_t[Size];
    memcpy(gcode, Data, Size);

    // reset static attributes
    GCode::reset();

    // main loop
    GCode* act = new GCode();

    uint8_t binary_length = act->computeBinarySize((char *) gcode);
    act->parseBinary(gcode, binary_length, false);

    delete(act);
    delete(GCodeSource::activeSource);
    delete[]gcode;
    return 0;
} */


extern "C" int LLVMFuzzerTestOneInput(const uint8_t *Data, size_t Size){
    if (! SizeIsOk(Size)) {
        return 0;
    }
	
    char test[4] = "hi!";
    #if DEBUG_OUTPUT
        std::cout << "Fuzzed Input: " << "Size: " << Size << "\n";
    #endif

    if(Size > 0 && Data[0] == 'H'){
    	if(Size > 1 && Data[1] == 'i'){
    	    if(Size > 2 && Data[2] == '!'){
    	    	test[5] = 0;
    	    }
    	}
    }
    
    return 0;
}
