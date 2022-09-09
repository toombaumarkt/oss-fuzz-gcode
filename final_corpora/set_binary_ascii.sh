#!/bin/bash



pos=$1
bit=$2
target=$3
path_to_testcases=$4

if ! ( [ $target -eq 0 ] || [ $target -eq 1 ] ); then
   echo "Target must be 1 or 0"
   exit 1
fi

for file in ./$path_to_testcases/* ./$path_to_testcases/**/*
do
   int=$(( 16#$(dd if="$file" bs=1 skip="$pos" count=1 2> /dev/null | xxd -p) ))
   
   (( mask = 1 << bit ))
   if (( (($int >> ($bit)) & 1) != $target )); then
   	(( int = int ^ $mask ))
   fi
   printf "%02x" "$int" | xxd -r -p | dd of="$file" bs=1 seek="$pos" conv=notrunc 2> /dev/null
done
