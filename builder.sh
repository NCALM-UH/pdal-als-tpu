#!/bin/bash

sudo rm -r -f build
mkdir build && cd build
cmake -D CMAKE_INSTALL_PREFIX=${CONDA_PREFIX} ..
cmake ..
make
sudo make install
cd ..

pdal --drivers | grep als_tpu