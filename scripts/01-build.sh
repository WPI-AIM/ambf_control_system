#! /bin/bash

rm -rf ../out/build/release/*
# cmake -S ../ -DCMAKE_BUILD_TYPE=Release -B ../out/build/release
time cmake -S ../ -G Ninja -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_C_COMPILER=clang -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_STANDARD=90 -DCMAKE_BUILD_TYPE=Release -B ../out/build/release

# rm -rf ../out/build/debug/*
# cmake -S ../ -DCMAKE_BUILD_TYPE=Debug-output --trace -B ../out/build/debug
