#!/bin/bash

project=$1
target=$2
path_to_crashes=$3

amount_of_crashes=0
crashing_files=""

for file in ./$path_to_crashes/*
do
   touch out.txt
   python3 ../infra/helper.py reproduce $project $target $file > out.txt
   if grep -q -e 'ERROR\|WARNING\|AddressSanitizer\|MemorySanitizer\|UndefinedBehaviourSanitizer' out.txt; then
      ((amount_of_crashes = amount_of_crashes + 1))
      crashing_files="${crashing_files}\n${file}"
   fi
done

echo ""
echo ""
echo "---------------RESULTS---------------"
echo -e "Crashing files: " $crashing_files
echo ""
echo "Crashes: " $amount_of_crashes
echo ""
