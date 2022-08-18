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


# install qt dependencies
RUN apt-get install -y bison build-essential clang flex gperf libatspi2.0-dev libbluetooth-dev libclang-dev libcups2-dev libdrm-dev libegl1-mesa-dev libfontconfig1-dev libfreetype6-dev libgstreamer1.0-dev libhunspell-dev libnss3-dev libopengl-dev libpulse-dev libssl-dev libts-dev libx11-dev libx11-xcb-dev libxcb-glx0-dev libxcb-icccm4-dev libxcb-image0-dev libxcb-keysyms1-dev libxcb-randr0-dev libxcb-render-util0-dev libxcb-shape0-dev libxcb-shm0-dev libxcb-sync-dev libxcb-util-dev libxcb-xfixes0-dev libxcb-xinerama0-dev libxcb-xkb-dev libxcb1-dev libxcomposite-dev libxcursor-dev libxdamage-dev libxext-dev libxfixes-dev libxi-dev libxkbcommon-dev libxkbcommon-x11-dev libxkbfile-dev libxrandr-dev libxrender-dev libxshmfence-dev libxshmfence1 llvm ninja-build nodejs python-is-python2 python2


# get qtbase
RUN wget https://download.qt.io/archive/qt/5.15/5.15.5/submodules/qtbase-everywhere-opensource-src-5.15.5.tar.xz
RUN tar -xvf qtbase-everywhere-opensource-src-5.15.5.tar.xz

# adding libc++ flag to build compatible QT libs
RUN cd $SRC/qtbase-everywhere-src-5.15.5/mkspecs && sed -i -e "s/QMAKE_CXXFLAGS    += -stdlib=libc++/QMAKE_CXXFLAGS    += -stdlib=libc++  \nQMAKE_CFLAGS += /g" linux-clang-libc++/qmake.conf && sed -i -e "s/QMAKE_LFLAGS      += -stdlib=libc++/QMAKE_LFLAGS      += -stdlib=libc++ -lpthread/g" linux-clang-libc++/qmake.conf

# build qtbase
RUN mkdir qt-build
RUN cd qt-build && $SRC/qtbase-everywhere-src-5.15.5/configure -confirm-license -opensource -platform linux-clang-libc++ -release -static -qt-zlib -qt-libjpeg -qt-libpng -qt-freetype -qt-pcre -qt-harfbuzz && make -j8 && make install


# get common testcases
RUN git clone --depth 1 https://github.com/toombaumarkt/gcode-fuzzing-testcases.git
RUN cp -r gcode-fuzzing-testcases/seed_corpus $SRC/

ADD parser $SRC/parser
ADD seed_corpus/ $SRC/seed_corpus
COPY build.sh $SRC/

ENV PATH="${PATH}:/usr/local/Qt-5.15.5/bin"
