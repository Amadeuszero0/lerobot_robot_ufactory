# Pika 遥操作 Piper（第一可用版）

本目录提供基于 LeRobot 的 **单个 Pika Sense 遥操作单台 Piper 机械臂** 的实验性实现。

当前保留的是项目开发过程中第一个能够实际跟随的版本：Piper 使用笛卡尔末端位姿 `MOVE P` 控制，Pika 位姿经过坐标轴映射后作为相对运动输入。本版本没有使用后续实验中的高频平滑滤波，也没有启用 CPV 控制。

> **重要提示**：本项目仍是实验版本，只适合在有人监护、低速、小范围、有物理急停和防坠措施的环境中测试。禁止直接用于无人值守、生产或大范围高速运动。

## 1. 当前支持范围

已经实际验证：

- 单个 Pika Sense 连接单台 Piper；
- Pika 的平移能够驱动 Piper 末端跟随；
- Pika 静止时，Piper 基本能够保持静止；
- Piper 通过 SocketCAN `can0` 通信；
- Pika 通过官方 Python SDK 通信，不依赖 ROS；
- 可使用无摄像头配置，减少 USB 占用。

尚未完成或尚未充分验证：

- 双 Pika 遥操作双 Piper；
- Piper 主从臂遥操作；
- CPV 连续速度模式；
- 长时间连续遥操作稳定性；
- 遥操作过程中同步采集完整 LeRobot 数据集；
- 突然断电、USB 掉线、CAN 掉线后的自动安全恢复。

## 2. 环境安装

环境和驱动请优先按照设备厂商及 LeRobot 的官方文档安装，不在本项目中复制或替代官方安装流程。

需要先完成：

1. 按 LeRobot 官方文档创建 Python/Conda 环境；
2. 按 Pika 官方手册安装 Pika Python SDK、定位驱动和基站相关依赖；
3. 按 Piper 官方手册安装 Piper SDK、USB-CAN 驱动及 `can-utils`；
4. 确认 Pika SDK 可以导入，Piper SDK 可以打开 `can0`；
5. 安装本仓库及 Piper 插件。

克隆项目：

```bash
git clone https://github.com/Amadeuszero0/lerobot_robot_ufactory.git
cd lerobot_robot_ufactory
```

进入按照官方文档创建好的环境，然后安装本项目：

```bash
pip install -e .
pip install -e ./piper
```

检查插件：

```bash
python -c "import lerobot_robot_ufactory_piper; print('piper plugin ok')"
```

> 本版本使用普通 `MOVE P` 控制，不需要为 CPV SDK 设置额外的 `PYTHONPATH`，也不要使用开发期间创建的 `piper_sdk_official_061` 覆盖环境。

## 3. 硬件连接要求

### 3.1 Pika

- Pika 尽量直接连接主机 USB 接口；
- 优先使用原装、质量可靠且支持数据传输的 USB 线；
- 不建议通过普通 USB Hub 连接；
- 当前示例配置使用 `/dev/ttyUSB50`，实际设备名可能不同；
- 单臂测试时，只连接当前使用的一个 Pika，避免定位器选择混乱。

检查设备：

```bash
ls -l /dev/ttyUSB*
ls -l /dev/serial/by-id/
ls -l /dev/serial/by-path/
```

如果实际端口不是 `/dev/ttyUSB50`，修改：

```text
piper/config/single_pika_piper_first_version_nocam.yaml
```

中的：

```yaml
teleop:
  port: /dev/ttyUSB50
```

### 3.2 USB-CAN 与 Piper

USB-CAN 应直接连接主机，不要接在不稳定的扩展坞或 USB Hub 上。项目测试中，扩展坞曾造成 CAN 发送队列积压和 `Message NOT sent`。

初始化 `can0`：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up
```

检查：

```bash
ip -details -statistics link show can0
tc -s qdisc show dev can0
candump -n 20 can0
```

正常情况下应满足：

- `can0` 为 `UP`；
- CAN 状态为 `ERROR-ACTIVE`；
- 波特率为 `1000000`；
- 能够收到 Piper 的周期反馈帧；
- 发送队列 `backlog` 不持续增长。

## 4. Pika 定位与基站标定

首次安装、基站位置发生变化或定位坐标异常时，重新执行：

```bash
uf-vive-calibrate --force-calibrate
```

使用 `--force-calibrate` 是为了忽略已有的旧标定结果，重新计算并保存当前两座基站之间的空间关系。只要基站的位置或朝向发生变化，就应重新标定。

标定输出中的关键内容包括：

```text
Global solve ... (acc err ...)
Position found for LH ... err: ...
```

建议两座基站的 `acc err` / `err` 均小于 `0.005 m`。标定时保证：

- 两座基站固定牢靠；
- 基站视野覆盖 Pika 的完整工作区域；
- Pika 定位器无遮挡；
- 标定完成后不要再移动基站。

检测 Pika 定位器：

```bash
python -c "from pika.sense import Sense; import time; s=Sense(port='/dev/ttyUSB50'); print('connect', s.connect()); time.sleep(8); print(s.get_tracker_devices()); s.disconnect()"
```

正常情况下最终应检测到：

```text
['LH0', 'LH1', 'T20']
```

其中：

- `LH0`、`LH1` 是两座 Lighthouse 基站；
- `T20`（编号可能不同）才是 Pika 上实际运动的定位器。

## 5. 启动前机械臂安全姿态

不要在 J2、J3 都接近 `0°` 的完全展开或奇异姿态下直接启动遥操作。

推荐先按 Piper 上的绿色拖动按钮，手动将机械臂摆到有明显弯曲、远离桌面和自身结构的安全姿态，再按一次绿色按钮退出拖动状态。

可参考的关节范围：

```text
J1： -20° ～ 20°
J2：  30° ～ 45°
J3： -50° ～ -30°
J4：  20° ～ 40°
J5： -10° ～ 10°
J6： -20° ～ 20°
```

曾经验证过的一组参考角度：

```text
[14.37, 37.08, -36.875, 26.538, 6.23, -10.153]
```

不要为了退出拖动模式调用：

```python
MotionCtrl_1(0x02, 0, 0)
```

该指令在当前设备上曾导致机械臂返回初始姿态。

## 6. 启动第一版遥操作

推荐使用无摄像头配置：

```text
piper/config/single_pika_piper_first_version_nocam.yaml
```

启动：

```bash
cd ~/lerobot_robot_ufactory

uf-piper-teleop \
  --config_path piper/config/single_pika_piper_first_version_nocam.yaml
```

看到：

```text
Press Enter to start teleop >>>
```

后按以下顺序操作：

1. 确认工作区内没有人员和障碍物；
2. 确认物理急停或电源开关触手可及；
3. 将 Pika 保持在舒适的中间位置并完全静止；
4. 按 Enter；
5. 继续保持 Pika 静止约 2 秒；
6. 第一次只缓慢平移 2～3 cm；
7. 确认方向正确、机械臂无快速跳变后，再逐渐扩大范围。

正常停止时只按一次：

```text
Ctrl+C
```

配置中使用：

```yaml
disable_torque_on_disconnect: false
```

目的是正常退出时不主动关闭关节力矩。但仍应在退出时托住机械臂前臂，并准备使用物理急停或断电。

## 7. 配置说明

第一版无摄像头配置的主要参数：

```yaml
robot:
  control_space: cartesian
  max_cartesian_step_mm: 1.0
  max_rotation_step_rad: 0.0002
  translation_deadband_mm: 1.5
  rotation_deadband_rad: 0.01
  workspace_x: [50, 600]
  workspace_y: [-500, 500]
  workspace_z: [50, 600]
  move_mode: move_p
  move_speed_percent: 5
  min_command_interval_s: 0.10
  disable_torque_on_disconnect: false
  cameras: {}

teleop:
  frequency: 20
  scale_xyz: 0.5
```

这些参数是当前硬件测试得到的保守值。不要在首次测试时提高速度、步长或缩放比例。

## 8. 当前版本的已知问题

### 8.1 移动时存在一帧一帧的抖动

Pika 静止时 Piper 基本不抖，但在平移或旋转手腕时，可以感觉到离散帧式运动。可能原因包括：

- `MOVE P` 位置指令本身是离散目标点；
- 控制频率、CAN 指令间隔与机械臂内部轨迹规划频率不完全匹配；
- Pika 定位数据存在少量噪声；
- 每帧发送新的笛卡尔目标，使 Piper 不断重新规划。

本版本特意没有使用后续实验中的低通平滑滤波。该滤波虽然减小了单次抖动幅度，但实际体验变成了幅度更小、频率更高的抖动，因此没有保留。

### 8.2 大范围平移可能中途停顿

当 Pika 一次平移约几十厘米时，Piper 可能移动一段距离后短暂停顿，再继续跟随。可能与以下因素有关：

- 最大单步限制；
- `MOVE P` 轨迹重复规划；
- Piper 实际位置与新目标之间的跟踪误差；
- 工作空间限制；
- CAN 发送节奏或队列状态。

建议当前版本只做小范围、低速、分段移动。

### 8.3 坐标方向依赖当前安装方式

Pika 到 Piper 的坐标映射是根据当前基站布置、Pika 握持方向和 Piper 安装方向标定得到的。如果出现“Pika 向右、Piper 向下”等现象，需要重新进行三个方向的纯平移测试并修改坐标映射矩阵。

以下变化都可能要求重新确认方向：

- 移动或旋转 Lighthouse 基站；
- 改变 Piper 底座方向；
- 改变 Pika 的握持方向或安装方向；
- 更换左右手设备；
- 重新执行基站空间标定。

### 8.4 当前只验证单臂

双臂需要额外解决：

- 两个 Pika 串口的稳定命名；
- 两个定位器 `Txx` 的左右手绑定；
- 两路 CAN 接口；
- 左右臂坐标映射；
- 双臂互相碰撞保护；
- USB 带宽和供电。

在完成这些内容前，不应直接复制单臂配置同时启动两条机械臂。

### 8.5 软件急停会导致机械臂下沉

开发过程中发现，发送 Piper 软件急停后，机械臂会进入类似阻尼/失力状态并在重力作用下下沉。因此：

- 不要把软件急停理解为机械抱闸；
- 触发软件急停前必须托住机械臂或设置机械防坠；
- 不要调用 `ResetPiper()` 作为普通恢复操作；
- 异常情况下优先使用硬件急停/电源，并避免人员进入机械臂下方。

## 9. 摄像头说明

无摄像头配置只用于遥操作调试，不保存相机画面。

项目中另有：

```text
piper/config/single_pika_piper_first_version.yaml
```

其中包含鱼眼相机和 Intel RealSense D435i RGB 配置示例，但设备路径、序列号、分辨率和 USB 带宽必须根据实际电脑修改。建议先在无摄像头模式下确认遥操作稳定，再逐个加入摄像头。

## 10. 故障排查

### `Message NOT sent`

检查：

```bash
tc -s qdisc show dev can0
ip -details -statistics link show can0
```

如果 `backlog` 持续增长：

- 停止遥操作程序；
- 将 USB-CAN 从扩展坞拔下；
- 直接连接主机；
- 重新初始化 `can0`；
- 再用 `candump` 确认收发。

### 只检测到 `LH0`、`LH1`

说明基站被识别，但 Pika 定位器还没有产生有效位姿。检查：

- Pika 定位器是否通电；
- USB 线是否支持稳定数据传输；
- 定位器是否被遮挡；
- 是否等待了足够长的初始化时间；
- 基站是否正常工作并完成标定。

### Piper 不跟随

依次确认：

1. Pika 位姿测试中 `T20` 的位置随手移动而变化；
2. `candump` 能收到 Piper 周期帧；
3. CAN 发送队列没有积压；
4. `Arm Status` 为正常状态；
5. 绿色拖动按钮已经关闭；
6. 使用的是 `single_pika_piper_first_version_nocam.yaml`；
7. 没有设置 CPV SDK 的额外 `PYTHONPATH`。

## 11. 安全声明

本项目为研究和实验用途。机械臂存在夹伤、碰撞、突然运动和断力下坠风险。使用者必须自行完成风险评估，并至少采取以下措施：

- 保证急停或断电开关触手可及；
- 初次测试使用低速、小步长和空载；
- 机械臂下方不得站人或放置贵重设备；
- 运行前清空工作空间；
- 停止和故障恢复时托住前臂；
- 不在机械臂关节缝隙附近放置手指；
- 未完成碰撞保护前禁止双臂同时运行。

