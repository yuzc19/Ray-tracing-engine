#!/usr/bin/env bash

SAMPLES=2000
METHOD=pt

# If project not ready, generate cmake file.
if [[ ! -d build ]]; then
    mkdir -p build
    cd build
    cmake ..
    cd ..
else
    rm -r build
    mkdir -p build
    cd build
    cmake ..
    cd ..
fi

# Build project.
cd build
make -j
cd ..

# Run all testcases. 
# You can comment some lines to disable the run of specific examples.
mkdir -p output
time bin/PA testcases/smallpt.txt new_output/smallpt.bmp $METHOD $SAMPLES
time bin/PA testcases/bunny.txt output/bunny.bmp $METHOD $SAMPLES
time bin/PA testcases/dof.txt output/dof.bmp $METHOD $SAMPLES
time bin/PA testcases/vase.txt output/vase.bmp $METHOD $SAMPLES
time bin/PA testcases/dispersion.txt output/no_dispersion.bmp $METHOD $SAMPLES
time bin/PA testcases/dispersion.txt output/dispersion.bmp $METHOD $SAMPLES dispersion
