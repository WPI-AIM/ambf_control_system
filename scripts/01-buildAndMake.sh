#! /bin/bash

rm -rf ../out/build/*
cmake -S ../ -B ../out/build/

cd ../out/build;  make -j$(nproc)
cd ../../