#!/usr/bin/env bash
# 一键构建 AVANTI2 6DOF 运行时库并跑 C++ 回归（docs phase1 §6）。
set -euo pipefail

cd "$(dirname "$0")/.."

BUILD_DIR="${1:-build/linux64}"

cmake -S . -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=Release \
  -DAVANTI_BUILD_TESTS=ON

cmake --build "$BUILD_DIR" --parallel

ctest --test-dir "$BUILD_DIR" --output-on-failure

echo "=== 产物 ==="
file "$BUILD_DIR/dist/libavanti2.so"
ldd "$BUILD_DIR/dist/libavanti2.so"
nm -D --defined-only "$BUILD_DIR/dist/libavanti2.so" | grep ' avanti_'
