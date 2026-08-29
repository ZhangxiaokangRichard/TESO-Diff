"""TESO-Diff Python 包。

Phase 2a 的全部 Python 运行时（bindings / controller / eso / simulation）
均在此包下。包内模块全部张量化，可对 B 个仿真实例一次解算，
Phase 2b Isaac/CUDA 后端直接平移复用（docs/TESO-Diff_phase2.md §1.4）。
"""
