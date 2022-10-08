#! /bin/bash

# cd ../out/build/release;  make -j$(nproc)
cd ../out/build/release; time ninja -j$(nproc)
ccache -s

# cd ../out/build/debug;  make -j$(nproc) VERBOSE=1;

cd ../../../
