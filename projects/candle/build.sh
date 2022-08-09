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

$SRC/qtbase-everywhere-src-5.15.5/configure -v -platform linux-clang-libc++ -prefix $SRC/Qt-5.15.5/ -static -opensource -confirm-license -release -no-compile-examples -nomake examples -nomake tests -qt-pcre -qt-zlib -qt-freetype -qt-harfbuzz -xcb -qt-libpng -qt-libjpeg -qt-sqlite && make -j $(($(nproc)+4)) && make install

# create seed_corpus.zip
zip -r $OUT/candle_$FUZZING_ENGINE\_seed_corpus.zip $SRC/seed_corpus/*


# build fuzzers
# e.g.
# $CXX $CXXFLAGS -std=c++11 -Iinclude \
#     /path/to/name_of_fuzzer.cc -o $OUT/name_of_fuzzer \
#     $LIB_FUZZING_ENGINE /path/to/library.a
#export QMAKE_CXX=$CXX
#export QMAKE_CXXFLAGS=$CXXFLAGS
#export QMAKE_CFLAGS=$CFLAGS
#export QMAKE_CC=$CC


$CXX $CXXFLAGS $SRC/parser/*.cpp $SRC/parser/tables/gcodetablemodel.cpp -o $OUT/candle_$FUZZING_ENGINE -I /usr/local/Qt-5.15.5/include/QtCore/ -I /usr/local/Qt-5.15.5/include/QtGui/ -I /usr/local/Qt-5.15.5/include/QtWidgets/ -I /usr/local/Qt-5.15.5/include/ /usr/local/Qt-5.15.5/plugins/imageformats/libqgif.a /usr/local/Qt-5.15.5/plugins/imageformats/libqico.a /usr/local/Qt-5.15.5/plugins/imageformats/libqjpeg.a /usr/local/Qt-5.15.5/lib/libqtlibjpeg.a /usr/local/Qt-5.15.5/lib/libQt5Widgets.a /usr/local/Qt-5.15.5/lib/libQt5Gui.a /usr/local/Qt-5.15.5/lib/libqtlibpng.a /usr/local/Qt-5.15.5/lib/libqtharfbuzz.a /usr/local/Qt-5.15.5/lib/libQt5Core.a /usr/local/Qt-5.15.5/lib/libqtpcre2.a $LIB_FUZZING_ENGINE -lpthread -fPIC
