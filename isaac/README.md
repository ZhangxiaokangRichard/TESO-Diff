# Isaac Sim / Isaac Lab 并行数据生成（Phase 2b）

Phase 2a 通过 Gate A 后，在数据/编码服务器（Ubuntu + Isaac Sim / Isaac Lab）
建立本目录下的并行环境。本阶段（phase2_sp1）只冻结目录规划，不编写实现。

## 规划结构（docs/TESO-Diff_phase2.md §7.3）

```text
isaac/
├── avanti_teso/
│   ├── __init__.py
│   ├── env_cfg.py
│   ├── env.py
│   ├── backend/           # base.py / so_backend.py / warp_backend.py
│   ├── wind/              # base.py / no_wind.py / uniform.py / thermal.py /
│   │                      # shear.py / gust.py / dynamic.py / composite.py
│   ├── control/           # primitive.py / scheduler.py
│   ├── task/              # base.py / circle.py / figure8.py / racetrack.py /
│   │                      # gps_triangle.py / projector.py
│   ├── sensors/           # gnss.py / imu.py / pitot.py / wind_estimator.py
│   ├── energy/            # accounting.py / label.py
│   ├── branch/            # snapshot.py / counterfactual.py
│   ├── dataset/           # schema.py / writer.py / shard.py / validate.py
│   └── tests/
└── scripts/
    ├── smoke_isaac.py
    ├── generate_dataset.py
    └── benchmark_env.py
```

## 关键约束

- Isaac 不负责固定翼气动：AVANTI2 的 aero/propulsion/6DOF/controller 全部由
  本项目自己的模型实现（docs §7.2）；
- 张量契约（docs §11）：state [B,Nx]、control [B,4]、wind [B,3]、time [B]；
- 严格按 1 → 8 → 32 → 128 → 512 → 1024+ env 扩展（docs §7.4）；
- Python 张量控制器（python/controller，Phase 2a 冻结）直接平移；
- CPU `.so` 与 GPU backend 的数值回归见 docs §43。
