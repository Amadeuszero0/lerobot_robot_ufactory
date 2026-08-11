# 历史 joint-stream 当前硬件测试配置

本目录提供从历史提交 `ed30ad0` 派生的当前硬件测试配置。历史快照原件仍保存在相邻的
`piper/` 目录中，没有被修改。

## 当前硬件绑定

| 设备 | 当前标识 |
|---|---|
| 左臂 Piper | `can_left` |
| 右臂 Piper | `can_right` |
| 左手 Pika 串口 | `/dev/pika_left` |
| 右手 Pika 串口 | `/dev/pika_right` |
| 左手 Vive Tracker | `LHR-818D4A5D` |
| 右手 Vive Tracker | `LHR-52C31F65` |

这些值与当前正式配置 `piper/config/dual_pika_piper.yaml` 一致。数据目录单独放在
`/home/star/lerobot_data/history_ed30ad0/` 下，避免和当前版本的数据混用。

## 配置文件

### 01：IK 只读预览

`01_dual_pika_piper_x_ik_preview_current_hardware.yaml`

- 不使能机械臂；
- `dry_run: true`；
- 不调用 `JointCtrl`、`EndPoseCtrl` 或 `GripperCtrl`；
- 只读取两臂 CAN 反馈，根据 Pika 动作计算并打印 `IK PREVIEW`。

这是测试历史版本时允许首先运行的配置。

### 02：带动力第一阶段

`02_dual_pika_piper_x_joint_stream_stage1_current_hardware_DANGER.yaml`

- 会使能两台 Piper；
- 会发送 `JointCtrl` 和夹爪命令；
- 历史实机测试曾出现严重抖动。

在模型检查、CAN 反馈检查和 01 预览输出没有全部通过人工复核前，不得运行 02。

## 使用重命名后的独立历史包

为了避免 Python 误加载当前版本，实际运行请使用相邻目录
`piper_history_ed30ad0/` 中已经重命名的独立包。不要再通过 `PYTHONPATH` 抢占同名模块。

在仓库根目录安装：

```bash
python -m pip install -e . --no-deps
python -m pip install -e \
  ./history/piper_joint_stream_20260809_ed30ad0/piper_history_ed30ad0 \
  --no-deps
```

确认导入的是独立历史模块：

```bash
python -c "import lerobot_robot_ufactory_piper_history_ed30ad0 as h; print(h.__file__)"
```

输出路径必须包含 `piper_history_ed30ad0/src/lerobot_robot_ufactory_piper_history_ed30ad0`。
安装历史包不会覆盖当前的 `lerobot_robot_ufactory_piper`；两者的发行名、模块名和命令行入口均不同。

实际运行配置以独立包内部的 `config/current_hardware/` 为准。完整测试流程必须逐步执行，
每一步检查结果后才能继续。

## 修复版配置位置

修复版位于独立包的 `config/current_hardware/`：

| 顺序 | 配置 | 行为 |
|---|---|---|
| 03 | `03_stabilized_joint_stream_preview.yaml` | 双臂只读模拟 |
| 04 | `04_stabilized_joint_stream_left_only_DANGER.yaml` | 仅左臂带动力 |
| 05 | `05_stabilized_joint_stream_right_only_DANGER.yaml` | 仅右臂带动力 |
| 06 | `06_stabilized_joint_stream_dual_DANGER.yaml` | 双臂带动力 |

原历史 `01/02` 保持 `stabilized_stream: false`，用于复现和对照；修复版 `03～06` 才启用
连续命令参考、速度/加速度限制、跟随误差保护和 IK 跳变保护。
带动力配置 `04～06` 退出时只断开通信，不解除关节使能，以保持机械臂当前姿态。
