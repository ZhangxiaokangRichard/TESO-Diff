#!/usr/bin/env bash
# Phase 2a 冒烟测试一键入口（docs §6.14 之前的接口级验证）：
#   portable 依赖审计 + 20s 回归 + 多实例并行 + 生命周期压力测试。
# 用法：
#   scripts/run_phase2a_smoke.sh              # 使用 runtime/fixedwing/lib 预构建 .so
#   scripts/run_phase2a_smoke.sh <library>    # 指定其他 .so（如 build/linux64/dist）
set -euo pipefail

cd "$(dirname "$0")/.."

LIB="${1:-runtime/fixedwing/lib/libavanti2.so}"
if [[ ! -f "$LIB" ]]; then
    echo "库不存在: $LIB（可先运行 scripts/build_all.sh）" >&2
    exit 1
fi

echo "=== [1/4] fixedwing 依赖审计（无 MATLAB） ==="
command python3 tests/regression/test_portable_bundle.py

echo "=== [2/4] 20s 逐样本回归 ==="
command python3 tests/regression/test_ctypes_20s.py --library "$LIB" --seconds 20 --print-every 5

echo "=== [3/4] 多实例并行正确性 ==="
command python3 tests/regression/test_ctypes_parallel.py \
    --library "$LIB" --instances 16 --seconds 20 --workers 16

echo "=== [4/4] 生命周期压力测试 ==="
command python3 tests/regression/test_ctypes_lifecycle.py \
    --library "$LIB" --warmup 1000 --cycles 5000

echo "=== Phase 2a 冒烟测试全部通过 ==="
