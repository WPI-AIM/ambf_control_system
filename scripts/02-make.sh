#! /bin/bash

cd ../out/build/release;  make -j$(nproc)
# cd ../out/build/debug;  make -j$(nproc) VERBOSE=1;

cd ../../../
