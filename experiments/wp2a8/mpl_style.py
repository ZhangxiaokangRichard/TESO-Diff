"""matplotlib 绘图统一配置（中文无乱码）与出图助手。

中文方案：优先使用系统 Noto CJK 字体（Noto Sans CJK SC → JP 回退），
并设置 axes.unicode_minus=False 避免负号显示为方块。
所有 experiments 出图必须调用 setup_chinese_font()。
"""

from __future__ import annotations

from pathlib import Path

import matplotlib

CJK_CANDIDATES = (
    "Noto Sans CJK SC",
    "Noto Sans CJK JP",
    "WenQuanYi Micro Hei",
    "Noto Serif CJK SC",
    "SimHei",
)

NOTO_TTC = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")


def setup_chinese_font() -> str:
    """配置 matplotlib 中文字体，返回实际使用的字体名。"""
    from matplotlib import font_manager

    if NOTO_TTC.exists():
        try:
            font_manager.fontManager.addfont(str(NOTO_TTC))
        except Exception:
            pass
    available = {f.name for f in font_manager.fontManager.ttflist}
    for name in CJK_CANDIDATES:
        if name in available:
            matplotlib.rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
            matplotlib.rcParams["axes.unicode_minus"] = False
            return name
    raise RuntimeError("no CJK font available; install fonts-noto-cjk")


def save_figure(fig, path: Path, dpi: int = 150) -> Path:
    """保存图像并关闭画布，返回路径。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=dpi, bbox_inches="tight")
    import matplotlib.pyplot as plt

    plt.close(fig)
    return path


def new_axes(rows: int = 1, cols: int = 1, figsize=(9.0, 5.0)):
    """创建带中文配置的 Figure/Axes。"""
    import matplotlib.pyplot as plt

    setup_chinese_font()
    fig, axes = plt.subplots(rows, cols, figsize=figsize)
    return fig, axes
