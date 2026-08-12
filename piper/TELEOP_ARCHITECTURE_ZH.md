# 双臂摇操两种控制链原理

## V7：固件笛卡尔控制

V7 分别读取左右 Pika Sense、Vive Tracker 和夹爪距离，通过持久化的 `LHR-*` 序列号固定左右手。
左右实测矩阵把 Tracker 世界坐标的平移和旋转映射到各自 PiPER-X 基座，随后组合成
`left.pose.*`、`right.pose.*` 和两侧夹爪目标。

`PiperFollower` 对每侧目标执行工作空间、死区、跟随误差和单周期限幅，然后调用 Piper SDK
的 `EndPoseCtrl`。机械臂的逆运动学与轨迹重规划由 Piper 固件完成。V7 还使用旋转/平移意图
门控：纯转腕时冻结 XYZ，大幅平移时即使手腕自然扰动也允许机械臂移动。

关键文件：

- `config/dual_pika_piper_x_full_speed_v7.yaml`
- `src/lerobot_robot_ufactory_piper/pika_teleop.py`
- `src/lerobot_robot_ufactory_piper/piper_follower.py`
- `src/lerobot_robot_ufactory_piper/motors/piper_motors_bus.py`
- `src/lerobot_robot_ufactory_piper/shared_vive_tracker.py`

## 师兄版：官方工具坐标与软件 IK

师兄版保留 Lerobot-Real 双臂摇操的三项核心：

1. PiPER-X 官方夹爪中心变换：`Rz(+90°) @ Ry(-90°) @ Tx(190 mm)`；
2. Pika 局部工具坐标相对位姿公式：`S_t = S_0 @ (P_0 C)^-1 @ (P_t C)`；
3. Pinocchio/CasADi 使用 Piper-X 官方 URDF 求解，并通过 `JointCtrl` 下发物理关节角。

为了不阻塞 50 Hz 主控制循环，左右 IK 各自在独立进程中运行，只保留最新目标。IK 无解、超出
URDF 关节限制或检测到自碰时，不发送新解而保持上一条有效关节目标。大于 30° 的首次分支变化
按约 1°、200 Hz 插值，降低启动跳变风险。Tracker 位姿沿用师兄版三帧中值/旋转中位样本滤波，
用于拒绝单帧跳点；每次重新启用摇操都会清空窗口并重新建立相对位姿原点。

关键文件：

- `config/dual_pika_piper_official_ik.yaml`
- `src/lerobot_robot_ufactory_piper/official_kinematics.py`
- `src/lerobot_robot_ufactory_piper/pika_teleop.py` 中的 `rotation_style: official`
- `src/lerobot_robot_ufactory_piper/piper_follower.py` 中的 `official_ik` 分支
- `src/lerobot_robot_ufactory_piper/motors/piper_motors_bus.py` 中的物理关节接口

## 并存原则

两套方案共用硬件发现、Pika 串口、共享 Vive 上下文与基本 Piper CAN 适配，但执行链由配置选择。
Pinocchio/CasADi 只在 `cartesian_command_mode: official_ik` 时延迟导入，因此 V7 和单臂终稿不依赖
师兄版环境，也不会改变原有参数或运动行为。
