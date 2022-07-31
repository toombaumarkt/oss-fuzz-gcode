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
RUN apt-get update && apt-get install -y make autoconf automake libtool
#RUN git clone --depth 1 <git_url> repetier-firmware     # or use other version control

# get common testcases
RUN git clone --depth 1 https://github.com/toombaumarkt/gcode-fuzzing-testcases.git
RUN cp -r gcode-fuzzing-testcases/seed_corpus $SRC/

COPY parser/ $SRC/
ADD seed_corpus/ $SRC/seed_corpus
ADD seed_corpus_binary/ $SRC/seed_corpus_binary
#WORKDIR repetier-firmware
COPY build.sh $SRC/
