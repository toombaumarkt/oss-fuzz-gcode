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

# build project
# e.g.
# ./autogen.sh
# ./configure
# make -j$(nproc) all

# copy gcode dictionary to OUT directoy
cp $SRC/gcode-fuzzing-testcases/gcode.dict $OUT/marlin_$FUZZING_ENGINE\_$SANITIZER.dict


# create seed_corpus.zip
zip -r $OUT/marlin_$FUZZING_ENGINE\_$SANITIZER\_seed_corpus.zip $SRC/seed_corpus/*

# build fuzzers
# e.g.
# $CXX $CXXFLAGS -std=c++11 -Iinclude \
#     /path/to/name_of_fuzzer.cc -o $OUT/name_of_fuzzer \
#     $LIB_FUZZING_ENGINE /path/to/library.a
$CXX $CXXFLAGS $SRC/wrapper.cpp $SRC/parser.cpp -o $OUT/marlin_$FUZZING_ENGINE\_$SANITIZER $LIB_FUZZING_ENGINE
