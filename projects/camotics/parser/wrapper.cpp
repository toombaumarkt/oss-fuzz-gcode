#include <sstream>

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

    Printer printer(std::cout, annotate);
    Parser(*input).parse(printer);

    delete(input);

    return 0;
}
