#!/bin/bash
rm -rf build
mkdir build
cd build
cmake -DOMPL_DIR=/usr/share/ompl/cmake/ ..
make
