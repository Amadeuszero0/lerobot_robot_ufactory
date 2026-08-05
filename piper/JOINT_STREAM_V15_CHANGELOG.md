# Pika -> Piper V15: joint-space streaming

Date: 2026-08-05

V15 is a NEW motion layer, isolated from the MOVE P path. V13/V14
(`uf::piper`, MOVE P) are completely untouched and still work.

## 1. 为什么原来的 MOVE P 有问题

原来的执行层把 Pika 的目标位姿直接发给固件的 `EndPoseCtrl`（MOVE P）。
固件每收到一个终点就**以固定速度重新规划一段轨迹**，由此产生三个症状：

1. **抖动**：每个控制周期（50 Hz）都在打断上一段轨迹、重新规划，
   机械臂表现为"匀速段 + 重规划"的走走停停。
2. **加速跟不上**：MOVE P 每段都是固定速度，无法匹配操作者的速度/加速度
   曲线；Pika 一加速，目标就跑在机械臂前面，只能追。
3. **不可控的中间过程**：笛卡尔目标到关节的逆解藏在固件里，我们无法在
   关节空间做平滑、连续性约束和逐关节限幅。

## 2. 新方法借鉴了哪里（官方 PikaAnyArm）

官方栈的平滑来自三层：

1. **自己做 IK**：`piper_IK.py` 用 pinocchio + URDF（`piper_description`）
   把笛卡尔目标解成关节角，而不是交给固件。
2. **关节空间流式控制**：`piper_ctrl_single_node.py` 用关节模式
   （`MotionCtrl_2(0x01, 0x01, speed, 0xad)` + `JointCtrl`）在 200 Hz 流式
   发关节位置，大步长时按 1 度插值。
3. **连续性种子**：IK 以上一解为初值，解保持连续，不会跳变。

V15 移植了 1 和 3，并把 2 简化为"50 Hz 每周期发一次关节命令 + 逐关节
限幅"（与主循环同频，先不上后台 200 Hz 插值线程；如果够顺就不用，不够
再加）。

> 说明：官方用 pinocchio + CasADi/IPOPT 做 IK；V15 用**纯 numpy 自实现**
> 的 URDF 运动学（数值雅可比 + Levenberg-Marquardt），零第三方依赖，
> 已在开发机自测：200 次随机小步增量，位置误差 < 0.6 mm、姿态 < 0.1°，
> 单次 IK 约 7 ms（50 Hz 预算 20 ms 内）。

## 3. 为什么新方法更好

- 关节空间是线性的：每个关节独立地向目标线性移动，不再有笛卡尔
  重规划的抖动来源。
- 我们直接控制每个关节每周期走多少度（`max_joint_step_deg`），相当于
  主动控制速度跟随，而不是等固件的固定速度段。
- 关节限位从 URDF 显式读取并在 IK 里裁剪；IK 残差超限时保持上一命令，
  不会乱发。
- 遥操层（Pika 映射、缩放、滤波）完全不变，只换执行层。

## 4. 移植了什么文件

- `piper/urdf/piper_description.urdf`：从官方仓库复制的 Piper URDF
  （13 KB，纯运动学，不需要 mesh）。
- `piper/src/lerobot_robot_ufactory_piper/piper_kinematics.py`：纯 numpy
  的 URDF 运动学（FK + 数值雅可比 + Levenberg-Marquardt IK），无第三方
  依赖（官方是 pinocchio/CasADi，我们不需要装任何东西）。
- `piper/src/lerobot_robot_ufactory_piper/piper_joint_stream.py`：新
  robot 类型 `uf::piper_joint_stream`（关节流 follower + 配置）。
- `piper/tools/verify_ik_fk.py`：**上真机前必跑**的 FK/IK 校验。
- `piper/config/single_pika_piper_joint_stream_v15.yaml`：V15 配置。
- `piper/src/lerobot_robot_ufactory_piper/__init__.py`：只追加了新类型的
  导入注册，不改任何已有行为。

## 5. 首次上机流程（重要，无额外依赖）

```bash
cd ~/lerobot_robot_ufactory
python piper/tools/verify_ik_fk.py --port can0
```

校验工具只读，不使能、不运动。它会把我们 FK 和 SDK 位姿对比，
选出匹配的末端帧候选（`ee_xyz_mm` / `ee_rpy_deg`）。**把它输出的最佳
候选写进 V15 配置**，然后才允许低速试跑：

```bash
uf-piper-teleop \
  --config_path=piper/config/single_pika_piper_joint_stream_v15.yaml
```

首轮测试要求：机械臂处于安全弯曲位姿、Pika 平移 < 5 cm、旋转 < 10°、
急停可及。如果 IK 校验显示误差很大，先不要跑，把输出贴回来。

## 6. 已知边界

- 50 Hz 关节流是第一步；若仍有可见台阶，再上"1°@200Hz 后台插值"。
- `ee_xyz_mm`/`ee_rpy_deg` 必须与你的 SDK 位姿帧匹配，否则方向和
  幅度都会错——这就是校验工具存在的原因。
- 夹爪路径复用现有 `set_gripper_percent`，与 MOVE P 路径一致。
