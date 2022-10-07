#! /bin/bash

rm -rf ../out/build/release/*
cmake -S ../ -DCMAKE_BUILD_TYPE=Release -B ../out/build/release

# rm -rf ../out/build/debug/*
# cmake -S ../ -DCMAKE_BUILD_TYPE=Debug-output --trace -B ../out/build/debug