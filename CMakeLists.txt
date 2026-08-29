# TESO-Diff 项目根 CMake 构建入口。
#
# 当前仅包含 phase1 冻结的 AVANTI2 6DOF 固定翼运行时（runtime/fixedwing）。
# Phase 2a 的控制器为纯 Python 张量实现（python/controller），不参与 CMake 编译；
# Phase 2b 若引入 Warp/Torch GPU backend，再在本文件扩展子目录。
#
# 构建：
#   cmake -S . -B build/linux64 -DCMAKE_BUILD_TYPE=Release -DAVANTI_BUILD_TESTS=ON
#   cmake --build build/linux64 --parallel
#   ctest --test-dir build/linux64 --output-on-failure
#
# 产物（与 phase1 路径风格保持一致，便于 python/bindings 默认查找）：
#   build/linux64/dist/libavanti2.so
#   build/linux64/dist/avanti_cpp_20s

cmake_minimum_required(VERSION 3.16)

project(teso_diff_runtime VERSION 0.1.0 LANGUAGES CXX)

option(AVANTI_BUILD_TESTS "Build native AVANTI validation executables" ON)

add_subdirectory(runtime/fixedwing)

if(AVANTI_BUILD_TESTS)
    enable_testing()
    add_executable(avanti_cpp_20s tests/cpp/cpp_20s_test.cpp)
    target_compile_features(avanti_cpp_20s PRIVATE cxx_std_11)
    target_link_libraries(avanti_cpp_20s PRIVATE avanti2)
    set_target_properties(avanti_cpp_20s PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/dist"
        BUILD_RPATH "$ORIGIN")
    add_test(NAME avanti_cpp_20s COMMAND avanti_cpp_20s)
endif()
