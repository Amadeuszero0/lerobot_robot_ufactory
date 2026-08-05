# piper 子项目说明（Pika / Piper 数据采集与遥操）

本目录是 `lerobot_robot_ufactory` 的**独立子项目**，复用父项目的 Pika、
相机和录制流程，加入 Piper 单臂/双臂 follower、leader、Pika 遥操与三种
双臂方案。**不改动父项目任何文件**。

## 目录结构

```text
piper/
├── config/         # 所有运行配置（YAML）
├── src/lerobot_robot_ufactory_piper/
│   ├── piper_follower.py     # MOVE P 执行层（step / direct）
│   ├── piper_joint_stream.py # 关节空间流式执行层
│   ├── piper_kinematics.py   # 纯 numpy URDF 运动学（FK/IK）
│   └── pika_teleop.py        # Pika 遥操（坐标映射、夹爪）
├── tools/          # 标定 / 测量 / 校验工具
├── urdf/           # Piper 运动学模型（纯运动学，无需 mesh）
└── README.md       # 本说明
```

## 三种双臂方案（config/）

| 方案 | 配置 | 说明 |
|---|---|---|
| Pika 直接采集 | `dual_pika_direct.yaml` | 双 Pika + 相机，不驱动机械臂 |
| Pika 遥操 Piper | `dual_pika_piper.yaml` | 双 Pika 控双 Piper |
| Piper 主从 | `dual_piper_leader_follower.yaml` | 双 leader 控双 follower |

## 三种单臂遥操执行模式

| 配置 | 模式 | 特点 | 建议 |
|---|---|---|---|
| `single_pika_piper_movep_step.yaml` | MOVE P 阶梯限幅 | 每周期小步，保守稳定 | 慢速精细任务 |
| `single_pika_piper_movep_direct_final.yaml` | MOVE P 直通 + 单周期大限幅 | 顺滑、跟手，**第一版终稿** | 日常采集（推荐） |
| `single_pika_piper_joint_stream.yaml` | 关节空间流式（自研 IK） | 理论最顺，绕开固件 IK | 进阶（先做手眼标定） |

`single_pika_piper.yaml` 是基础模板（占位符）。

## 安装

```bash
cd /path/to/lerobot_robot_ufactory
pip install -e .
pip install -e ./piper
```

CAN 口按 Piper 官方方式配置（1 Mbps）：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up
```

## 运行

```bash
# 单臂遥操（在仓库根目录）
uf-piper-teleop --config_path=piper/config/single_pika_piper_movep_direct_final.yaml

# 录制
uf-piper-record --config_path=piper/config/dual_pika_piper.yaml
```

## 标定与工具（tools/）

| 工具 | 用途 | 什么时候做 |
|---|---|---|
| `uf-vive-calibrate --force-calibrate` | 基站（灯塔）位置标定 | 首次 / 挪动基站后 |
| `tools/calibrate_tracker_offset.py` | Pika 定位器到夹持中心的偏移（`tracker_to_robot_eef`） | 换安装方式 / 扭腕有假平移时 |
| `tools/measure_pika_piper_mapping.py` | 测量 Pika 轴向与当前映射 | 挪动基站布局后 |
| `tools/verify_ik_fk.py --calibrate` | joint_stream 前的手眼标定（基座+末端帧） | joint_stream 首次使用前 |

## 坐标映射说明

- **平移**：原始 tracker（灯塔世界系）增量经实测矩阵
  `_RAW_TO_PIPER_TRANSLATION` 直接映射到 Piper 基座系（前→+X、右→−Y、
  上→+Z），由 `tools/measure_pika_piper_mapping.py` 测出。
- **旋转**：校准矩阵 + 工具轴修正，方向经实机验证。
- **夹爪**：Pika 开口距离（0≈闭合，100≈全开）直接映射到 Piper 夹爪。
- `tracker_to_robot_eef` 平移偏移仍为 `[0,0,0]`；若扭腕时机械臂有明显
  假平移，用 `calibrate_tracker_offset.py` 标定后写入。

## 安全

- 默认断开不卸力（保持姿态）；`disable_torque_on_disconnect: false`。
- `movep_direct_final` 连接后会自动移动到 `startup_tcp_pose`，运行前确认
  周围无障碍。
- 首次测试：Pika 平移 < 5 cm、旋转 < 10°，急停保持可及。
- direct 模式有跟随误差上限（600 mm / 3.2 rad）和单周期大限幅
  （25 mm / 0.35 rad），防止快速动作导致固件重规划卡顿。

## 版本历史（简）

- 早期实验版本（V1~V13 等）已清理，只保留有区分度的三种模式。
- `movep_step`：50 Hz MOVE P 阶梯限幅，1:1 映射，较快跟随。
- `movep_direct_final`：MOVE P 直通 + 单周期大限幅（**第一版终稿**）。
- `joint_stream`：关节空间流式（纯 numpy IK），摆脱 MOVE P 重规划抖动；
  使用前先跑 `verify_ik_fk.py --calibrate`。

## 第三方声明

见 `THIRD_PARTY_NOTICES.md`（Piper SDK / 官方 LeRobot 适配等）。
