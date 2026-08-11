# Pika / Piper 遥操作配置说明

`config` 目录保留了当前硬件需要的稳定入口、专项测试入口和少量回退配置。运行实机前应先检查配置中的 CAN、Pika 串口与 Tracker 序列号。

## 完全未修改的原始快照

最初的 Piper/Pika 实现保存在以下 Git 引用中，未包含后续修改：

- 分支：`original-untouched-piper`
- 标签：`original-untouched-piper-20260801`
- 精确提交：`45f9d9f`

该快照仍包含原始占位设备名，例如 `can_follower1`、`can_follower2` 和 `/dev/REPLACE_*`。它只作为历史源文件；需要在当前硬件上运行时，应另建运行配置，不要修改归档引用。

## 当前稳定双臂配置

配置：`config/dual_pika_piper.yaml`

- 使用 `MOVE_P + EndPoseCtrl`，由 Piper 固件完成逆解；
- 使用左右固定 Tracker 序列号和已经验证的夹爪控制链路；
- 响应较快、动作幅度较大，是专项测试失败时的回退版本。

```bash
uf-piper-teleop --config_path=piper/config/dual_pika_piper.yaml
```

## 左右实测平移配置

配置：`config/dual_pika_piper_measured_translation.yaml`

- 左右 Pika 分别使用 2026-08-11 实测平移矩阵；
- `scale_xyz` 为 `0.50`；
- 55% 速度、夹爪控制和退出保持参数与当前测试基线一致。

## PiPER-X J5 只读预览

配置：`config/dual_pika_piper_x_j5_preview.yaml`

- 只读取 Pika 与双臂反馈；
- 使用正确的 PiPER-X URDF 计算目标 J4/J5/J6；
- 不发送任何运动、模式或夹爪命令；
- 日志直接显示 `J5=当前->目标(变化量)`。

```bash
uf-piper-teleop \
  --config_path=piper/config/dual_pika_piper_x_j5_preview.yaml
```

## PiPER-X J5 实机测试

配置：`config/dual_pika_piper_x_j5_test.yaml`

- 保留实测平移、55% 速度和稳定夹爪参数；
- 左右使用各自的旋转实测矩阵；
- 横向摆腕主要映射到末端局部 Y 轴，从而驱动 PiPER-X J5；
- 仍走稳定的 `MOVE_P` 链路，不启用发生过严重抖动的关节流。

必须先确认只读预览方向正确，再运行：

```bash
uf-piper-teleop \
  --config_path=piper/config/dual_pika_piper_x_j5_test.yaml
```

完整诊断见 `PIPER_X_J5_DIAGNOSIS_ZH.md`。

## 保留的低速腕部轴配置

配置：`config/dual_pika_piper_verified_axes.yaml`

- 保留提交 `153d283` 时的低速保护参数；
- Piper 速度 40%，笛卡尔限步 6 mm，旋转限步 0.025 rad；
- 用于回退比较，不再作为 PiPER-X J5 问题的修复方案。

```bash
uf-piper-teleop \
  --config_path=piper/config/dual_pika_piper_verified_axes.yaml
```

## 硬件预检

配置：`config/dual_pika_piper_preflight.yaml`

用于低幅度硬件检查。运行前仍需确认左右 CAN、Pika 串口、Tracker 绑定和机械臂安全位姿。

## 稳定单臂配置

配置：`config/single_pika_piper_setting.yaml`

双臂专项调整没有修改该配置。

```bash
uf-piper-teleop \
  --config_path=piper/config/single_pika_piper_setting.yaml
```

## 已否决的实验

PiPER-X 软件 IK + `JointCtrl` 关节流在实机上出现过严重抖动。当前只允许把它用于 `dry_run` 只读 IK 预览，不应直接恢复带动力关节流。TCP 补偿或不同腕部坐标系的旧实验仍可从 Git 历史恢复，但未经重新审查不能用于实机。
