#!/bin/bash
#rm -rf build bin lib
rm -rf build lib
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j "$(getconf _NPROCESSORS_ONLN)"
#sudo make install



