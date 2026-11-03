#!/usr/bin/env python3
"""fixedwing runtime 依赖审计：验证 runtime/fixedwing 源码不依赖已安装的 MATLAB。

TESO-Diff 结构（对应 docs/TESO-Diff_phase2.md §6.2）：
    runtime/fixedwing/include/   生成模型头 + avanti_c_api.h 公共头
    runtime/fixedwing/generated/ Embedded Coder 生成源码（不含 C ABI bridge）
    runtime/fixedwing/wrapper/   手写 C ABI bridge（avanti_c_api.{h,cpp}）

检查内容：
1. 必需文件齐全（生成源码、solver 头文件、C ABI bridge）；
2. 所有带引号的 #include 都能在 fixedwing 目录内闭合
   （rtw_continuous.h 中的 tmwtypes.h 位于 MATLAB_MEX_FILE 条件分支，
    standalone CMake 构建不定义该宏，实际走同目录 rtwtypes.h，属白名单例外）；
3. 无 Windows 盘符绝对路径、/usr/local/MATLAB、matlabroot 或 -lmw/-lmat/-lmex 链接。

在 Ubuntu 目标机上不装 MATLAB 直接运行：
    python3 tests/regression/test_portable_bundle.py
"""

from __future__ import annotations

import re
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
FIXEDWING_ROOT = PROJECT_ROOT / "runtime" / "fixedwing"
INCLUDE_ROOT = FIXEDWING_ROOT / "include"
GENERATED_ROOT = FIXEDWING_ROOT / "generated"
WRAPPER_ROOT = FIXEDWING_ROOT / "wrapper"


def main() -> int:
    # 1) 必需文件清单：缺任何一个都说明打包不完整
    required = [
        GENERATED_ROOT / "AVANTI2_ertmodel0827.cpp",
        GENERATED_ROOT / "AVANTI2_ertmodel0827_data.cpp",
        WRAPPER_ROOT / "avanti_c_api.cpp",
        INCLUDE_ROOT / "AVANTI2_ertmodel0827.h",
        INCLUDE_ROOT / "rtw_continuous.h",
        INCLUDE_ROOT / "rtw_solver.h",
        INCLUDE_ROOT / "rtwtypes.h",
        INCLUDE_ROOT / "avanti_c_api.h",
    ]
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise AssertionError("missing fixedwing files: " + ", ".join(missing))

    files = (
        sorted(GENERATED_ROOT.glob("*"))
        + sorted(WRAPPER_ROOT.glob("*"))
        + sorted(INCLUDE_ROOT.glob("*"))
    )
    quoted_include = re.compile(r'^\s*#\s*include\s+"([^"]+)"', re.MULTILINE)
    # (所在文件, 被包含头) 白名单：该 include 只在 MATLAB_MEX_FILE 下生效
    inactive_host_only_includes = {("rtw_continuous.h", "tmwtypes.h")}
    unresolved: list[str] = []
    forbidden: list[str] = []
    # 禁止出现的 MATLAB 依赖特征
    forbidden_patterns = (
        re.compile(r"[A-Za-z]:[\\/]"),  # Windows 盘符绝对路径
        re.compile(r"/usr/local/MATLAB", re.IGNORECASE),  # MATLAB 安装绝对路径
        re.compile(r"matlabroot", re.IGNORECASE),  # MATLAB 根目录宏
        re.compile(r"(?:^|\s)-l(?:mw|mat|mex|mx)\w*", re.IGNORECASE),  # MathWorks 链接库
    )

    for path in files:
        if not path.is_file():
            continue
        text = path.read_text(encoding="utf-8", errors="replace")
        # 2) 逐个检查带引号 include 是否在 fixedwing 目录内可解析
        for header in quoted_include.findall(text):
            if (path.name, header) in inactive_host_only_includes:
                # 该 include 位于 MATLAB_MEX_FILE 分支下；standalone CMake 构建
                # 不定义该宿主仿真宏，会走相邻的 rtwtypes.h 分支，因此跳过。
                continue
            if not (path.parent / header).is_file() and not (INCLUDE_ROOT / header).is_file():
                unresolved.append(f"{path.name}: {header}")
        # 3) 检查源码内容中的 MATLAB 依赖特征
        for pattern in forbidden_patterns:
            if pattern.search(text):
                forbidden.append(f"{path.name}: {pattern.pattern}")

    # CMakeLists.txt（根目录与 runtime/fixedwing）同样不得引用 MATLAB 安装路径或链接库
    cmake_text = (PROJECT_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    cmake_text += "\n" + (FIXEDWING_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    for pattern in forbidden_patterns:
        if pattern.search(cmake_text):
            forbidden.append(f"CMakeLists.txt: {pattern.pattern}")

    if unresolved:
        raise AssertionError("unresolved private includes: " + ", ".join(unresolved))
    if forbidden:
        raise AssertionError("installed-runtime dependency found: " + ", ".join(forbidden))

    print(f"fixedwing_files={len(files)} unresolved_quoted_includes=0")
    print("absolute_matlab_paths=0 matlab_link_libraries=0")
    print("Fixedwing runtime dependency audit passed")
    att_root=PROJECT_ROOT / "runtime/fixedwing_att"
    att_include=att_root / "include"
    att_files=[p for part in ("include","generated","wrapper") for p in (att_root/part).glob("*") if p.is_file()]
    assert len(att_files)==19, "Attitude bundle is incomplete"
    for path in att_files:
        text=path.read_text(encoding="utf-8",errors="replace")
        for header in quoted_include.findall(text):
            if (path.name,header) in inactive_host_only_includes: continue
            assert (path.parent/header).is_file() or (att_include/header).is_file(), (path,header)
        for pattern in forbidden_patterns:
            assert not pattern.search(text), (path,pattern.pattern)
    for cmake_file in (att_root/"CMakeLists.txt",PROJECT_ROOT/"runtime/common/avanti_runtime.cmake"):
        for pattern in forbidden_patterns:
            assert not pattern.search(cmake_file.read_text(encoding="utf-8")),cmake_file
    print(f"fixedwing_att_files={len(att_files)} standalone include/link audit passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
