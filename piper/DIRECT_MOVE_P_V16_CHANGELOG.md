# Pika -> Piper V16: direct MOVE P (ported from lerobot_real)

Date: 2026-08-05

V16 ports the execution improvements from the senior `lerobot_real` fork
while keeping our verified Pika mapping. V13/V14 (step mode) are untouched.

## 为什么师兄版本效果好的分析

1. **direct 命令模式（核心）**：我们的 step 模式每周期只允许目标前进
   6mm/0.10rad，产生"阶梯追赶 + 走走停停"；师兄版本把**完整目标**直接
   发给 `EndPoseCtrl`，只在"目标离当前太远"（600mm/3.2rad）时才拒绝，
   机械臂以 30% 速度连续飞向真实目标——没有人为阶梯，看起来顺。
2. **帧数学**：师兄用官方约定的逆变换
   `tracker_to_robot_eef: [-190,0,0,0,90,0]`（`Ry(-90)@Tx(190)` 的逆）把
   Pika 目标直接换算到 Piper 原生 J6 帧，不用我们那套两次拟合矩阵。
3. **可复现启动**：`move_to_tcp_pose` 先把机械臂移到配置的
   `robot_base_pose`（他们机器实测位姿），每次会话起点一致，参考点采样
   可靠。
4. **健壮性**：使能前等真实反馈（`wait_for_follower_feedback`）、每周期
   `assert_follower_ready`、非有限值/工作空间检查、断连时保持位姿。
5. 50Hz + 30% 速度 + `scale_xyz 1.0`。

## V16 移植内容（默认全关，V13/V14 行为不变）

- `cartesian_command_mode`（step/direct，默认 step）+ 跟随误差上限
  `max_cartesian_following_error_mm` / `max_rotation_following_error_rad`。
- `startup_tcp_pose` + `startup_move_timeout_s`：连接后自动移动到安全起始
  位姿（仅在配置了该字段时）。
- `hold_position_on_disconnect`：断连（不卸力）时先发一次保持命令。
- `feedback_startup_timeout_s`：使能前等待真实反馈帧。
- 保留我们验证过的 Pika 映射（`use_raw_translation_mapping` + 校准旋转），
  不照搬师兄的帧参数（那是他们机器的标定值）。

## 首测

```bash
uf-piper-teleop \
  --config_path=piper/config/single_pika_piper_direct_v16.yaml
```

注意：连接后机械臂会**自动移动到 startup_tcp_pose**（若配置），确认周围
无障碍再运行；急停保持可及。Pika 平移 < 5cm、旋转 < 10° 起步。
