#!/bin/bash -eu
# Copyright 2022 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
################################################################################


# copy gcode dictionary to OUT directoy
cp $SRC/gcode-fuzzing-testcases/gcode.dict $OUT/repetier_firmware_ascii_$FUZZING_ENGINE\_$SANITIZER.dict


# create seed_corpus.zip
zip -r $OUT/repetier_firmware_binary_$FUZZING_ENGINE\_$SANITIZER\_seed_corpus.zip $SRC/seed_corpus_binary/*
zip -r $OUT/repetier_firmware_ascii_$FUZZING_ENGINE\_$SANITIZER\_seed_corpus.zip $SRC/seed_corpus_ascii/*


# build fuzzers
# parseBinary build
$CXX $CXXFLAGS -DFUZZ_TARGET=1 -std=c++17 $SRC/wrapper.cpp $SRC/gcode.cpp -o $OUT/repetier_firmware_binary_$FUZZING_ENGINE\_$SANITIZER $LIB_FUZZING_ENGINE

# parseAscii build
$CXX $CXXFLAGS -DFUZZ_TARGET=0 -std=c++17 $SRC/wrapper.cpp $SRC/gcode.cpp -o $OUT/repetier_firmware_ascii_$FUZZING_ENGINE\_$SANITIZER $LIB_FUZZING_ENGINE
