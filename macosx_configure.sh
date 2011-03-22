#!/bin/sh
##

mkdir -p build || exit 1
cd build || exit 1

cmake .. \
    -Wno-dev \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_VERBOSE_MAKEFILE=1 \
    $*

echo "Program configured in directory build."
echo "Now go into build/ and run make."
