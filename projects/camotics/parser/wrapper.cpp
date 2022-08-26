#include <sstream>
#include <utility>
#include <iostream>
#include <cstring>
#include "wrapper.h"

#include <gcode/Printer.h>
#include <gcode/parse/Parser.h>

using namespace std;
using namespace cb;
using namespace GCode;


bool SizeIsOk(size_t Size) {
    return Size >= MIN_DATA_SIZE && Size <= MAX_DATA_SIZE;
}

extern "C" int LLVMFuzzerTestOneInput(const uint8_t *Data, size_t Size){
    if(! SizeIsOk(Size)) {
        return 0;
    }

    bool annotate = false;
    cb::InputSource *input = new InputSource((const char*) Data, Size);

    #if not DEBUG_OUTPUT
        std::ostream null_output(nullptr);
    #endif

    // redirecting output to null when not in debug mode
    Printer printer(
        #if DEBUG_OUTPUT
        std::cout,
        #else
        null_output,
        #endif
        annotate
    );
    

    try
    {
        //std::cout << fuzzed_input << "\n";
        Parser(*input).parse(printer);
        //std::cout << "END \n";
    }
    catch(const std::exception& e)
    {
        #if DEBUG_OUTPUT
            std::cerr << e.what() << '\n';
        #endif
    }
    
    delete(input);


    return 0;
}
