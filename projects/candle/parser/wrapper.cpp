#include <sstream>

#include "wrapper.h"

#include "gcodeparser.h"
#include "gcodepreprocessorutils.h"
#include "gcodeviewparse.h"


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
    if(! SizeIsOk(Size)) {
        return 0;
    }

    std::stringstream ss = ConvertByteDataToStringStream(Data, Size);

    GcodeViewParse gvp;

    std::string line;
    QList<QString> commands;

    while(std::getline(ss, line)) {
        commands.append(line.c_str());
    }

    gvp.toObjRedux(commands, 5, 5);

    return 0;
}

