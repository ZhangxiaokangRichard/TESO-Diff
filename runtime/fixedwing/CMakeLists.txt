# AVANTI2 6DOF 固定翼运行时库构建（TESO-Diff Phase 2a）。
#
# 来源：phase1 项目 ~/AVANTI_6DOFmodel（Embedded Coder R2021a 生成的
# AVANTI2_ertmodel0827 C++ class + 手写稳定 C ABI bridge）。
# ABI 版本 0x00010000，固定步长 0.02 s（fixed-step ODE4，50 Hz）。
#
# 目录约定（对应 docs/TESO-Diff_phase2.md §6.2）：
#   include/    公共头文件（生成模型头 + avanti_c_api.h）
#   generated/  Embedded Coder 生成源码（模型 + rt 运行时），不含 C ABI bridge
#   wrapper/    手写 C ABI bridge（avanti_c_api.h / avanti_c_api.cpp）
#   lib/        预构建产物 libavanti2.so（phase1 Ubuntu 构建，无 MATLAB 依赖）
#
# 若模型端口、参数、初值或步长变化，必须在 R2021a 机器重新生成
# AVANTI2_ertmodel0827.cpp / *_data.cpp 并同步 include/，然后重跑全部回归。

set(AVANTI_FIXEDWING_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")
set(AVANTI_INCLUDE_DIR "${AVANTI_FIXEDWING_ROOT}/include")
set(AVANTI_GENERATED_DIR "${AVANTI_FIXEDWING_ROOT}/generated")
set(AVANTI_WRAPPER_DIR "${AVANTI_FIXEDWING_ROOT}/wrapper")

set(AVANTI_MODEL_SOURCES
    "${AVANTI_GENERATED_DIR}/AVANTI2_ertmodel0827.cpp"
    "${AVANTI_GENERATED_DIR}/AVANTI2_ertmodel0827_data.cpp"
    "${AVANTI_GENERATED_DIR}/rtGetInf.cpp"
    "${AVANTI_GENERATED_DIR}/rtGetNaN.cpp"
    "${AVANTI_GENERATED_DIR}/rt_nonfinite.cpp"
    "${AVANTI_WRAPPER_DIR}/avanti_c_api.cpp")

foreach(required_file IN LISTS AVANTI_MODEL_SOURCES)
    if(NOT EXISTS "${required_file}")
        message(FATAL_ERROR
            "Missing fixedwing source ${required_file}. "
            "Copy portable sources from the phase1 AVANTI_6DOFmodel project "
            "(interface/ and portable/avanti2/).")
    endif()
endforeach()

add_library(avanti2 SHARED ${AVANTI_MODEL_SOURCES})
target_compile_features(avanti2 PUBLIC cxx_std_11)
target_compile_definitions(avanti2 PRIVATE AVANTI_BUILDING_DLL)
target_include_directories(avanti2
    PUBLIC
        "$<BUILD_INTERFACE:${AVANTI_INCLUDE_DIR}>"
        "$<INSTALL_INTERFACE:include>")
set_target_properties(avanti2 PROPERTIES
    OUTPUT_NAME "avanti2"
    POSITION_INDEPENDENT_CODE ON
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/dist"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/dist"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/dist")

if(WIN32)
    set_target_properties(avanti2 PROPERTIES PREFIX "")
endif()

if(MINGW)
    target_link_options(avanti2 PRIVATE -static -static-libgcc -static-libstdc++)
endif()

include(GNUInstallDirs)
install(TARGETS avanti2
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES "${AVANTI_INCLUDE_DIR}/avanti_c_api.h"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
