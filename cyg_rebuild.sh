#! /bin/bash
#
clear
printf "DSD-FME Digital Speech Decoder - Florida Man Edition
Automatic Git Pull and Rebuild for Cygwin\n\n"
sleep 1
##Open your clone folder##
git pull
sleep 2
##cd into your build folder##
cd build
cmake ..
make -j $(nproc)
make install
