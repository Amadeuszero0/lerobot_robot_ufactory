# PiPER-X J5 向外摆腕问题：原因、预览与测试

## 结论

当前稳定双臂配置没有锁住 J5，也没有把 J5 的范围缩小。问题来自旋转动作的轴分配：

- 稳定链路发送 `EndPoseCtrl(X, Y, Z, RX, RY, RZ)`，由机械臂固件完成逆解；
- 项目里的 PiPER-X URDF 不参与这条 `MOVE_P` 链路，只参与软件 IK 预览或关节流实验；
- 2026-08-11 的 Pika 实测数据显示，“向右摆腕”经过旧映射后主要落在末端局部 Z 轴；
- PiPER-X 在当时左右实际姿态下，J5 的末端局部转轴都接近 +Y，而局部 Z 基本对应 J6；
- 因而手腕动作很大时，固件主要使用 J6，J5 只产生较小的补偿运动，夹爪仍保持朝内。

当时两臂的数值为：

| 机械臂 | J5 末端局部转轴 | 旧“向右摆腕”命令的主要分量 |
|---|---|---|
| 左臂 | `[+0.073, +0.997, 0]` | 局部 `+Z` |
| 右臂 | `[-0.181, +0.984, 0]` | 局部 `+Z` |

新配置使用左右 Pika 各自的三组实测旋转数据重新拟合：

- 低头动作映射到末端局部 `-X`；
- 向右摆腕映射到末端局部 `+Y`，主要驱动 J5；
- 绕手柄轴滚转映射到末端局部 `+Z`，主要驱动 J6。

左手向左摆腕是上述第二项的反方向，所以应使左臂 J5 减小；右手向右摆腕应使右臂 J5 增大。这也正好让初始约 `左 +16.5° / 右 -17.2°` 的镜像姿态朝相反方向展开。

用监测日志里的真实初始关节角和 PiPER-X 模型离线复算，完整实测手势得到：

| 动作 | 映射后的末端局部旋转 | IK 预计 J5 |
|---|---:|---:|
| 左手向左摆腕 | Y 轴 `-25.18°` | `+16.50° -> -12.57°` |
| 右手向右摆腕 | Y 轴 `+40.10°` | `-17.20° -> +27.71°` |

两次 IK 残差都小于 `0.00007`。这说明新矩阵在 PiPER-X 模型中能够产生明确的 J5 外展动作；实机测试仍需从小角度开始，因为完整右手实测动作会对应约 `45°` 的 J5 变化。

## 第一步：只读预览

此步骤不发送任何运动或夹爪命令：

```bash
cd ~/lerobot_robot_ufactory
conda activate uf_lerobot

git pull --ff-only \
  https://gh-proxy.com/https://github.com/Amadeuszero0/lerobot_robot_ufactory.git \
  main

python -m pip install -e ./piper --no-deps

uf-piper-check-config \
  piper/config/dual_pika_piper_x_j5_preview.yaml \
  piper/config/dual_pika_piper_x_j5_test.yaml

uf-piper-teleop \
  --config_path=piper/config/dual_pika_piper_x_j5_preview.yaml \
  2>&1 | tee /tmp/piper_x_j5_preview.log
```

按回车开始后，保持手柄位置基本不变，依次只做以下动作，每个动作保持约两秒：

1. 左 Pika 向左摆腕；
2. 左 Pika 回到中立；
3. 右 Pika 向右摆腕；
4. 右 Pika 回到中立；
5. 左右各做一次绕手柄轴滚转，用来确认滚转仍主要进入 J6。

日志示例：

```text
IK PREVIEW left_piper_x_j5_preview ... J5=16.50->2.10(-14.40)deg ...
IK PREVIEW right_piper_x_j5_preview ... J5=-17.20->-1.80(+15.40)deg ...
```

预期结果：

- 左臂向左摆腕时，`J5` 括号内变化量明显为负；
- 右臂向右摆腕时，`J5` 括号内变化量明显为正；
- 幅度应明显超过噪声，建议绝对值至少约 `5°`；
- `residual` 应小于配置中的 `0.02`，不能连续出现 IK 保持警告。

提取关键日志：

```bash
grep -E "IK PREVIEW|residual|ERROR" /tmp/piper_x_j5_preview.log | tail -n 160
```

## 第二步：实机 MOVE_P 测试

只有只读预览方向和幅度都正确时才运行：

```bash
uf-piper-teleop \
  --config_path=piper/config/dual_pika_piper_x_j5_test.yaml \
  2>&1 | tee /tmp/piper_x_j5_test.log
```

该配置仍使用已经验证速度和夹爪稳定的 `MOVE_P` 路径，保留：

- 左右独立实测平移矩阵；
- `scale_xyz: 0.50`；
- `move_speed_percent: 55`；
- 原夹爪滤波、限步和退出保持参数。

第一次实机测试时，先只转动一侧约 10～15°，观察对应 J5，再逐渐增加。不要让身体或手进入机械臂下方；如果方向与预览不一致、机械臂突跳或异常抖动，立即 `Ctrl+C`。退出后本配置保持当前位姿和力矩。

## 如何判断后续该改哪里

- 只读预览中 J5 已有正确的大幅变化，实机仍不动：问题位于 PiPER-X 固件的 `EndPoseCtrl` 逆解/解支选择；继续修改 Pika 矩阵无效。要确定控制 J5，只能重新解决安全的 PiPER-X 关节控制链路。
- 只读预览中 J5 方向相反：只需对对应侧矩阵的第二行取反，不需要改平移、速度或夹爪。
- 只读预览中 J5 仍很小：需要重新采集“纯摆腕”数据，避免同时滚转手柄；不能靠增大 `rotation_scale` 掩盖轴串扰。
- 只读预览和实机都正确：将本测试配置作为新的双臂候选版本，再分别微调左右 `rotation_scale`。
