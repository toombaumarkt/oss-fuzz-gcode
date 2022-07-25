#include <iostream>
#include <fstream>
#include <cstring>
#include <stdio.h>
#include <vector>

#include "wrapper.h"

// Declare system global variable structure
system_t sys; 

bool SizeIsOk(size_t Size) {
    return Size >= MIN_DATA_SIZE && Size <= MAX_DATA_SIZE;
}

extern "C" int LLVMFuzzerTestOneInput(const uint8_t *Data, size_t Size){
    if(! SizeIsOk(Size)) {
        return -1;
    }

    gc_init();
    
    protocol_main_loop(Data, Size);
    return 0;
}