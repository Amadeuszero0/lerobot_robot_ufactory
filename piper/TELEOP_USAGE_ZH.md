# Pika–PiPER-X 摇操使用说明

本目录只维护三套正式摇操配置：

| 方案 | 配置文件 | 控制方式 | 状态 |
|---|---|---|---|
| 双臂 V7 | `config/dual_pika_piper_x_full_speed_v7.yaml` | `MOVE_P + EndPoseCtrl` | 当前实机验证良好，日常首选 |
| 双臂师兄版 | `config/dual_pika_piper_official_ik.yaml` | 官方工具坐标 + Pinocchio/CasADi IK + `JointCtrl` | 新集成，必须先低幅测试 |
| 单臂终稿 | `config/single_pika_piper_setting.yaml` | bounded direct `MOVE_P` | 保留原成功版本，不改参数 |

## 1. 拉取与安装

```bash
cd ~/lerobot_robot_ufactory
git -c http.proxy= pull --ff-only origin main
conda activate uf_lerobot
python -m pip install -e . --no-deps
python -m pip install -e ./piper --no-deps
```

V7 和单臂终稿不需要 Pinocchio/CasADi。只有师兄版需要：

```bash
conda install -n uf_lerobot -c conda-forge \
  pinocchio=4.1.0 casadi=3.7.2
```

不要用普通 PyPI `pin` 替代：师兄版需要 `pinocchio.casadi`。检查环境：

```bash
python - <<'PY'
import casadi
import pinocchio
from pinocchio import casadi as cpin
print("official IK dependencies: OK")
PY
```

师兄版配置默认使用以下 Piper-X 官方 URDF：

```text
/home/star/piper_runtime/agx_arm_description/agx_arm_urdf/piper_x/urdf/piper_x_description.urdf
```

运行前确认它存在：

```bash
test -f /home/star/piper_runtime/agx_arm_description/agx_arm_urdf/piper_x/urdf/piper_x_description.urdf \
  && echo "Piper-X URDF: OK"
```

如果实际位置不同，只修改师兄版配置两侧的 `ik_urdf_path` 和
`ik_package_dir`，不要修改 V7。

## 2. 通用硬件预检

确认机械臂周围净空、急停可触达，并停止其他 Piper/Pika 控制进程。然后运行：

```bash
cd ~/lerobot_robot_ufactory
python piper/tools/diagnostics/check_dual_teleop_hardware.py \
  piper/config/dual_pika_piper_x_full_speed_v7.yaml
```

这个工具只读取两个 CAN、两个 Pika 串口、两个 Tracker 和夹爪数据，输出中应有：

```json
"robot_commands_sent": false
```

左右硬件固定绑定为：

| 侧别 | Pika | Tracker | Piper CAN |
|---|---|---|---|
| 左 | `/dev/pika_left` | `LHR-818D4A5D` | `can_left` |
| 右 | `/dev/pika_right` | `LHR-52C31F65` | `can_right` |

如果更换 Tracker，重新运行：

```bash
python piper/tools/diagnostics/identify_dual_pika_trackers.py --duration-s 5
```

## 3. 启动双臂 V7（日常推荐）

```bash
uf-piper-teleop \
  --config_path=piper/config/dual_pika_piper_x_full_speed_v7.yaml \
  2>&1 | tee /tmp/dual_pika_piper_v7.log
```

V7 特点：100% `MOVE_P` 速度、旋转比例 0.60、左右实测映射、平移意图优先门控。
它是当前已验证版本。启动后回车前保持 Pika 静止；首次动作依次测试单侧小幅平移、
单侧小幅旋转、夹爪，确认后再进行双臂大幅动作。

## 4. 启动师兄版双臂

先用同一配置执行只读预检：

```bash
python piper/tools/diagnostics/check_dual_teleop_hardware.py \
  piper/config/dual_pika_piper_official_ik.yaml
```

确认 URDF 与依赖均正常后运行：

```bash
uf-piper-teleop \
  --config_path=piper/config/dual_pika_piper_official_ik.yaml \
  2>&1 | tee /tmp/dual_pika_piper_official_ik.log
```

这套方案使用师兄项目的关键控制链：

1. `tracker_to_robot_eef: [-190, 0, 0, -90, 0, -90]` 表示 PiPER-X 官方夹爪中心到 J6 的逆变换；
2. Pika 相对运动按官方局部工具坐标公式生成 J6 目标；
3. 两个独立 IK 工作进程使用 Piper-X 官方 URDF 求关节角；
4. 有效解通过 `JointCtrl` 下发，无解、越限或自碰时保持上一条有效目标。

它与 V7 的固件 `MOVE_P` 链路不同，第一次只能做 3–5° 小角度和 2–3 cm 小平移。
出现关节跳变、抖动、`over_limit=True` 持续不恢复时立即停止，继续使用 V7。

## 5. 启动原成功单臂版本

该文件保持原样，仍使用原先的 `can0`、`/dev/ttyUSB0` 和 Tracker 配置：

```bash
uf-piper-teleop \
  --config_path=piper/config/single_pika_piper_setting.yaml \
  2>&1 | tee /tmp/single_pika_piper.log
```

如果当前机器只提供 `can_left` 和 `/dev/pika_left`，不要直接修改正式终稿；先复制一份现场配置，
再只替换 `robot.port`、`teleop.port` 和 `tracker_device_id`。

## 6. 停止和日志

- 正常退出：`Ctrl+C`；
- 不要在机械臂没有支撑时配置退出自动失能；
- V7 日志：`/tmp/dual_pika_piper_v7.log`；
- 师兄版日志：`/tmp/dual_pika_piper_official_ik.log`；
- 单臂日志：`/tmp/single_pika_piper.log`。

只读查看机械臂反馈：

```bash
python piper/tools/diagnostics/monitor_piper.py --port can_left
python piper/tools/diagnostics/monitor_piper.py --port can_right
```
