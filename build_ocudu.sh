#!/bin/bash

set -e

sudo usermod -a -G render $USER
sudo usermod -a -G video $USER

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
TARGET_DIR="$SCRIPT_DIR/build"
#TARGET_DIR="/home/$USER/ocudu/build"

echo "--- Create build: $TARGET_DIR ---"
mkdir -p "$TARGET_DIR"
cd "$TARGET_DIR"
rm -rf ./*

export CXX=/opt/rocm/bin/hipcc
export HSA_OVERRIDE_GFX_VERSION=10.3.0

echo "--- Run CMake ---"
cmake -DCMAKE_PREFIX_PATH="/opt/rocm-7.2.0;/usr" \
      -Dyaml-cpp_DIR=/usr/lib/x86_64-linux-gnu/cmake/yaml-cpp \
      -Dhip_DIR=/opt/rocm-7.2.0/lib/cmake/hip \
      -DCMAKE_HIP_ARCHITECTURES=gfx1030 \
      -DBUILD_TESTS=ON \
      ..

echo "--- Build ---"
make -j$(nproc) ocudu_ldpc
make -j$(nproc) ldpc_decoder_benchmark


echo "--- Running Benchmark ---"
cd "tests/benchmarks/phy/upper/channel_coding/ldpc"

#if ./ldpc_decoder_benchmark; then
#    echo "--- Benchmark completed successfully! ---"
#else
#    echo "--- Benchmark failed! ---"
#    exit 1
#fi

#echo "--- DONE! ---"

