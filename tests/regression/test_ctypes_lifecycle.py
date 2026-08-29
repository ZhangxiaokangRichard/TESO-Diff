#!/usr/bin/env python3
"""生命周期压力测试：反复 create → tick → reset → tick → destroy，验证进程资源有界。

默认先预热 1000 个周期，再实测 10000 个周期；要求 RSS 增长不超过 16 MiB，
且进程句柄数（Windows）/ 文件描述符数（Linux）不增加。
Windows 通过 psapi.GetProcessMemoryInfo 读工作集，Linux 读 /proc/self/statm 与
/proc/self/fd，均不依赖 psutil。

用法（Windows）：
    python tests/test_ctypes_lifecycle.py --library build/win64/dist/avanti2.dll \
        --warmup 1000 --cycles 10000
"""

from __future__ import annotations

import argparse
import ctypes as ct
import gc
import os
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "python" / "bindings"))

from avanti import AvantiInput, AvantiLibrary  # noqa: E402


def resident_bytes() -> int:
    """返回当前进程常驻内存（RSS）字节数；Windows 与 Linux 分别实现。"""
    if sys.platform == "win32":
        # 使用 psapi.GetProcessMemoryInfo 查询工作集大小
        from ctypes import wintypes

        class ProcessMemoryCounters(ct.Structure):
            _fields_ = [
                ("cb", wintypes.DWORD),
                ("PageFaultCount", wintypes.DWORD),
                ("PeakWorkingSetSize", ct.c_size_t),
                ("WorkingSetSize", ct.c_size_t),
                ("QuotaPeakPagedPoolUsage", ct.c_size_t),
                ("QuotaPagedPoolUsage", ct.c_size_t),
                ("QuotaPeakNonPagedPoolUsage", ct.c_size_t),
                ("QuotaNonPagedPoolUsage", ct.c_size_t),
                ("PagefileUsage", ct.c_size_t),
                ("PeakPagefileUsage", ct.c_size_t),
            ]

        counters = ProcessMemoryCounters()
        counters.cb = ct.sizeof(counters)
        kernel32 = ct.WinDLL("kernel32", use_last_error=True)
        psapi = ct.WinDLL("psapi", use_last_error=True)
        kernel32.GetCurrentProcess.restype = wintypes.HANDLE
        psapi.GetProcessMemoryInfo.argtypes = [
            wintypes.HANDLE,
            ct.POINTER(ProcessMemoryCounters),
            wintypes.DWORD,
        ]
        psapi.GetProcessMemoryInfo.restype = wintypes.BOOL
        process = kernel32.GetCurrentProcess()
        if not psapi.GetProcessMemoryInfo(
            process, ct.byref(counters), counters.cb
        ):
            raise ct.WinError(ct.get_last_error())
        return int(counters.WorkingSetSize)

    # Linux：/proc/self/statm 第二字段为常驻页数
    statm = Path("/proc/self/statm")
    if statm.is_file():
        resident_pages = int(statm.read_text(encoding="ascii").split()[1])
        return resident_pages * os.sysconf("SC_PAGE_SIZE")
    raise RuntimeError("RSS measurement is implemented for Windows and Linux")


def resource_count() -> int:
    """返回进程资源计数：Windows 为进程句柄数，Linux 为打开的 fd 数。"""
    if sys.platform == "win32":
        from ctypes import wintypes

        count = ct.c_ulong()
        kernel32 = ct.WinDLL("kernel32", use_last_error=True)
        kernel32.GetCurrentProcess.restype = wintypes.HANDLE
        kernel32.GetProcessHandleCount.argtypes = [
            wintypes.HANDLE,
            ct.POINTER(wintypes.DWORD),
        ]
        kernel32.GetProcessHandleCount.restype = wintypes.BOOL
        process = kernel32.GetCurrentProcess()
        if not kernel32.GetProcessHandleCount(process, ct.byref(count)):
            raise ct.WinError(ct.get_last_error())
        return int(count.value)
    fd_dir = Path("/proc/self/fd")
    return len(list(fd_dir.iterdir())) if fd_dir.is_dir() else 0


def exercise(library: AvantiLibrary, cycles: int) -> None:
    """每个周期：创建实例 → tick → reset → tick → 销毁，检验无泄漏。"""
    inputs = AvantiInput(throttle_01=0.5)
    for _ in range(cycles):
        model = library.model(0.0)
        first = model.tick(inputs, 0.0)
        if not first.finite():
            raise RuntimeError("non-finite output during lifecycle test")
        model.reset(10.0)
        second = model.tick(inputs, 10.0)
        if not second.finite():
            raise RuntimeError("non-finite output after reset")
        model.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--library", type=Path, default=None)
    parser.add_argument("--warmup", type=int, default=1000)
    parser.add_argument("--cycles", type=int, default=10000)
    parser.add_argument("--max-rss-growth-mib", type=float, default=16.0)
    args = parser.parse_args()

    library = AvantiLibrary(args.library)
    # 预热：让 CRT/分配器等运行时惰性初始化稳定下来
    exercise(library, args.warmup)
    gc.collect()
    rss_before = resident_bytes()
    resources_before = resource_count()

    # 正式测量周期
    exercise(library, args.cycles)
    gc.collect()
    rss_after = resident_bytes()
    resources_after = resource_count()
    growth = max(0, rss_after - rss_before)
    limit = int(args.max_rss_growth_mib * 1024 * 1024)

    print(f"warmup={args.warmup} measured_cycles={args.cycles}")
    print(f"rss_before={rss_before} rss_after={rss_after} growth_bytes={growth}")
    print(
        f"resources_before={resources_before} resources_after={resources_after} "
        f"growth={resources_after - resources_before}"
    )
    # 判定：RSS 增长受限，且句柄/fd 数不得增加（存在泄漏会暴露在这里）
    if growth > limit:
        raise AssertionError(f"RSS growth {growth} exceeds limit {limit}")
    if resources_after > resources_before:
        raise AssertionError("process handle/file-descriptor count increased")
    print("Python lifecycle stress test passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
