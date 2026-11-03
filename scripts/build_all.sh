#!/usr/bin/env bash
# Native Ubuntu build. Usage: scripts/build_all.sh [build-dir] [all|fixedwing|fixedwing_att]
set -euo pipefail
cd "$(dirname "$0")/.."
BUILD_DIR="${1:-build/linux64}"
RUNTIME="${2:-all}"
OLD=ON; ATT=ON
case "$RUNTIME" in
  all) ;; fixedwing) ATT=OFF ;; fixedwing_att) OLD=OFF ;;
  *) echo "runtime must be all, fixedwing or fixedwing_att" >&2; exit 2 ;;
esac
cmake -S . -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release -DAVANTI_BUILD_TESTS=ON \
  -DAVANTI_BUILD_FIXEDWING="$OLD" -DAVANTI_BUILD_FIXEDWING_ATT="$ATT" -DAVANTI_STAGE_LIBS=ON
cmake --build "$BUILD_DIR" --parallel
ctest --test-dir "$BUILD_DIR" --output-on-failure
for model in avanti2 avanti2_att; do
  [[ "$model" == avanti2 && "$OLD" == OFF ]] && continue
  [[ "$model" == avanti2_att && "$ATT" == OFF ]] && continue
  file "$BUILD_DIR/dist/lib${model}.so"
  ldd "$BUILD_DIR/dist/lib${model}.so"
  nm -D --defined-only "$BUILD_DIR/dist/lib${model}.so" | grep ' avanti_'
done
if [[ "$ATT" == ON ]]; then
  EXTRA=()
  [[ "$OLD" == OFF ]] && EXTRA+=(--skip-old-runtime)
  python3 tests/regression/test_att_runtime.py --library "$BUILD_DIR/dist/libavanti2_att.so" "${EXTRA[@]}"
fi
