#!/usr/bin/env bash

ROUNDS=1000
PHOTONS=10000000
CKPT_INTERVAL=50
METHOD=sppm

# If project not ready, generate cmake file.
if [[ ! -d build ]]; then
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
mkdir -p sppm_output
time bin/PA testcases/bunny.txt sppm_output/bunny_inter.bmp $METHOD $ROUNDS $PHOTONS $CKPT_INTERVAL
time bin/PA testcases/china.txt sppm_output/china.bmp $METHOD $ROUNDS $PHOTONS $CKPT_INTERVAL
time bin/PA testcases/heart.txt sppm_output/heart.bmp $METHOD $ROUNDS $PHOTONS $CKPT_INTERVAL