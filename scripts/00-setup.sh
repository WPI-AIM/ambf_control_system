#! /bin/bash
pwd=$(pwd)

rm -rf ../out
mkdir -p ../out/build/release
mkdir -p ../out/build/debug

# Build rbdl package to get rbdl_config.h 
rbdl_build_path=../out/make_dir/rbdl_build
mkdir -p $rbdl_build_path
time cmake -S ../Sway/vendor/rbdl/ -G Ninja -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_C_COMPILER=clang -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_STANDARD=90 -B $rbdl_build_path
cd $rbdl_build_path; 

time ninja -j$(nproc)

cd $pwd
