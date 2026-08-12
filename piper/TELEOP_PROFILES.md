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

## 左腕连续性修复 V2 只读预览

配置：`config/dual_pika_piper_x_j5_left_fix_preview_v2.yaml`

- 两侧均为 `dry_run`，不会启用力矩或发送运动/夹爪命令；
- 左侧使用最近正交旋转矩阵，去除旧三点拟合矩阵的剪切；
- 与旋转锁实机候选一致，在转腕时冻结 XYZ；
- dry-run IK 使用上一帧有效目标连续求解，并报告/拒绝腕部解支跳变；
- 日志包含 Pika 原始/映射旋转、目标 XYZ/轴角、J5 近零与 branch step。

它是当前左腕“先右后左”问题的诊断入口，不是实机控制配置：

```bash
uf-piper-teleop \
  --config_path=piper/config/dual_pika_piper_x_j5_left_fix_preview_v2.yaml \
  2>&1 | tee /tmp/piper_x_left_fix_preview_v2.log
```

## 左腕连续性修复 V3：单左臂低速实机

配置：`config/single_pika_piper_x_left_wrist_v3.yaml`

- 只连接左臂 `can_left` 和左 Pika，右臂完全不连接；
- 继续使用稳定的 `MOVE_P + EndPoseCtrl`，不启用实机抖动过的关节流；
- `scale_xyz: 0.0`，首轮完全禁止 Pika 平移；
- 左臂速度为 10%，单次旋转目标最多 0.025 rad；
- 夹爪关闭，左腕旋转比例降为 V2 的一半；
- 必须确认急停可用、机械臂周围净空，并先做 5 度小角度动作。

```bash
uf-piper-teleop \
  --config_path=piper/config/single_pika_piper_x_left_wrist_v3.yaml \
  2>&1 | tee /tmp/piper_x_left_wrist_v3.log
```

## 左腕连续性修复 V4：55% 速度验证

配置：`config/single_pika_piper_x_left_wrist_full_speed_v4.yaml`

- 相对 V3 只把 `move_speed_percent` 从 10 提高到当前实机基线 55；
- 仍然只连接左臂，Pika 平移与夹爪保持关闭；
- 旋转比例仍为 0.30，单次旋转目标限制仍为 0.025 rad；
- 用于判断提高 MOVE_P 速度是否会放大末端轻微晃动。

```bash
uf-piper-teleop \
  --config_path=piper/config/single_pika_piper_x_left_wrist_full_speed_v4.yaml \
  2>&1 | tee /tmp/piper_x_left_wrist_full_speed_v4.log
```

## V5 全功能双臂 55% 版本

配置：`config/dual_pika_piper_x_full_v5.yaml`

- 双臂均使用稳定的 `MOVE_P + EndPoseCtrl`，运行速度 55%；
- 恢复左右实测平移、夹爪控制及旋转时冻结 XYZ；
- 左侧使用 V2/V3/V4 已验证的新正交矩阵，右侧保持稳定矩阵；
- 两侧旋转比例统一为 0.30，并关闭高频旋转诊断日志。

```bash
uf-piper-teleop \
  --config_path=piper/config/dual_pika_piper_x_full_v5.yaml \
  2>&1 | tee /tmp/dual_pika_piper_x_full_v5.log
```

## V6 全功能双臂运动意图版本

配置：`config/dual_pika_piper_x_full_motion_v6.yaml`

- 保留 V5 的双臂 `MOVE_P`、55% 速度、实测平移、旋转矩阵和夹爪；
- 旋转冻结门控新增平移速度判定；
- 控制点平移速度达到 20 mm/s 时，即使手腕同时转动也允许 XYZ 跟随；
- 只有“旋转快、平移慢”的纯转腕仍会冻结 XYZ，避免转腕串入平移。

```bash
uf-piper-teleop \
  --config_path=piper/config/dual_pika_piper_x_full_motion_v6.yaml \
  2>&1 | tee /tmp/dual_pika_piper_x_full_motion_v6.log
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
