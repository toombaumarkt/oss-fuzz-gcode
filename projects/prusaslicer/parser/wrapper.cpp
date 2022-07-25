#include <iostream>
#include <fstream>
#include <string.h>
#include <stdio.h>
#include <vector>

#include "wrapper.h"
#include "GCodeReader.hpp"

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

extern "C" int LLVMFuzzerTestOneInput(const uint8_t *Data, size_t Size){
    if(! SizeIsOk(Size)){
        return -1;
    }

    Slic3r::GCodeReader* parser = new Slic3r::GCodeReader();

    parser->reset();

    int line_count = 0;
    
    std::stringstream input = ConvertByteDataToStringStream(Data, Size);
    std::string gcode_line;

    while(getline(input, gcode_line)){
        parser->parse_line(gcode_line, [&line_count](Slic3r::GCodeReader& reader, const Slic3r::GCodeReader::GCodeLine& line) {
        #if DEBUG_OUTPUT
            std::cout << "Line " << ++line_count << ": " << "CMD: " << line.cmd(); 
            if (line.has_x())
                std::cout << " X: " << line.x();
            if (line.has_y())
                std::cout << " Y: " << line.y();
            if (line.has_z())
                std::cout << " Z: " << line.z();
            if (line.has_e())
                std::cout << " E: " << line.e();
            if (line.has_f())
                std::cout << " F: " << line.f();
            if (! line.comment().empty())
                std::cout << " CMT: " << line.comment();
            if (line.has_unknown_axis())
                std::cout << " UNKNOWN AXIS: " << line.value(UNKNOWN_AXIS);
            std::cout << "\n";
        #endif
    });
    }

    delete(parser);
    return 0;
}
