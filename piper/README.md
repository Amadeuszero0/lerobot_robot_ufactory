# Piper 摇操子项目

本目录维护 Pika 遥操作 PiPER-X 的正式实现。当前只提供三套运行配置：

| 配置 | 用途 |
|---|---|
| `config/dual_pika_piper_x_full_speed_v7.yaml` | 已验证的 V7 双臂 `MOVE_P` 版本，日常首选 |
| `config/dual_pika_piper_official_ik.yaml` | 从师兄 Lerobot-Real 集成的官方工具坐标 + 软件 IK 双臂版本 |
| `config/single_pika_piper_setting.yaml` | 原成功单臂终稿，参数保持不变 |

完整安装、硬件预检和启动命令见 [TELEOP_USAGE_ZH.md](TELEOP_USAGE_ZH.md)。

两种双臂方案的原理和关键代码见
[TELEOP_ARCHITECTURE_ZH.md](TELEOP_ARCHITECTURE_ZH.md)。

## 目录

```text
piper/
├── config/                       # 三套正式运行配置
├── src/lerobot_robot_ufactory_piper/
│   ├── pika_teleop.py            # Pika 映射、V7 门控、师兄版官方坐标公式
│   ├── piper_follower.py         # MOVE_P 与 official_ik 两种执行分支
│   ├── official_kinematics.py    # 师兄版 Pinocchio/CasADi IK 工作进程
│   ├── piper_joint_stream.py     # 保留的旧 joint-stream 实现，不属于正式配置
│   └── piper_kinematics.py       # 保留的旧纯 NumPy 运动学
├── tools/diagnostics/            # 当前维护的只读诊断工具
├── tests/                        # 正式配置与保留运动学测试
└── urdf/                         # 保留的旧本地运动学 URDF
```

## 快速启动

```bash
# V7 双臂
uf-piper-teleop \
  --config_path=piper/config/dual_pika_piper_x_full_speed_v7.yaml

# 师兄版双臂（需先安装 Pinocchio/CasADi 并准备 Piper-X 官方 URDF）
uf-piper-teleop \
  --config_path=piper/config/dual_pika_piper_official_ik.yaml

# 原成功单臂
uf-piper-teleop \
  --config_path=piper/config/single_pika_piper_setting.yaml
```

## 只读硬件检查

```bash
python piper/tools/diagnostics/check_dual_teleop_hardware.py \
  piper/config/dual_pika_piper_x_full_speed_v7.yaml
```

该工具不会使能机械臂，也不会发送运动或夹爪命令。
