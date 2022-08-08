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

FROM gcr.io/oss-fuzz-base/base-builder
RUN apt-get update && apt-get install -y build-essential cmake make autoconf automake libtool+

# install build-dep https://askubuntu.com/questions/496549/error-you-must-put-some-source-uris-in-your-sources-list
RUN cp /etc/apt/sources.list /etc/apt/sources.list~ && sed -Ei 's/^# deb-src /deb-src /' /etc/apt/sources.list && apt-get update
RUN apt-get build-dep -y qtbase5-dev

# build qtbase
#RUN wget https://download.qt.io/archive/qt/5.15/5.15.5/submodules/qtbase-everywhere-opensource-src-5.15.5.tar.xz
#COPY qtbase-everywhere-opensource-src-5.15.5.tar.xz .
ADD qtbase-everywhere-src-5.15.5 $SRC/qtbase-everywhere-src-5.15.5
#RUN tar -xvf qtbase-everywhere-opensource-src-5.15.5.tar.xz

RUN $SRC/qtbase-everywhere-src-5.15.5/configure -v -platform linux-clang-libc++ -prefix $SRC/Qt-5.15.5/ -static -opensource -confirm-license -release -no-compile-examples -nomake examples -nomake tests -qt-pcre -qt-zlib -qt-freetype -qt-harfbuzz -xcb -qt-libpng -qt-libjpeg -qt-sqlite && make -j $(($(nproc)+4)) && make install


#RUN $SRC/qtbase-everywhere-src-5.15.5/configure -v -opensource -confirm-license -prefix $SRC/Qt-5.15.5 -release -static -no-compile-examples -nomake examples -nomake tests -no-opengl -no-widgets -- -DCMAKE_CXX_FLAGS_RELEASE="-O1" -DQT_USE_DEFAULT_CMAKE_OPTIMIZATION_FLAGS=ON VERBOSE=1 cmake --build . --parallel

#ADD Qt-5.15.5 $SRC/Qt-5.15.5

# get common testcases
RUN git clone --depth 1 https://github.com/toombaumarkt/gcode-fuzzing-testcases.git
RUN cp -r gcode-fuzzing-testcases/seed_corpus $SRC/

ADD parser $SRC/parser
ADD seed_corpus/ $SRC/seed_corpus
#WORKDIR grbl
COPY build.sh $SRC/

ENV PATH="${PATH}:${SRC}/Qt-5.15.5/bin"
