# CtrlZ 与本框架的观测及控制对齐检查（2026-10-20）

对照入口为 `/home/mayh/MYH/Project_code/C++/CtrlZ/example/BitbotSimulation/config_AMP_Depth_VAE_CA.json`。
本次只读取 CtrlZ 源码和模型，没有修改该仓库。没有运行其完整 UI/调度器闭环。

## 比较前提：模型不同

当前 CtrlZ JSON 指向训练目录
`2026-10-13_15-09-45_unitree_g1_amp_walk_target_depth_casa_noinityaw/exported/policy.onnx`，
并非本框架的 `example/exported/g1_test`。实际文件 SHA256：

| 模型 | 完整 policy.onnx SHA256 |
| --- | --- |
| CtrlZ 配置所指模型 | `3394dda829a48293e20991f1046f7cc84ab21531fdeb248310f12c76a5b8b984` |
| g1_test | `7eb343b957841cd1334557ad037fa26c35d568e8bb71cbc7ccce3096a1fb27ec` |

因此此前两边“右偏程度不同”的现象不能直接用于判断推理框架的误差，需要先用同一模型、
相同命令和场景比较。用户已确认将先核对模型进行测试。

## IMU：发现并修正了真实坐标错误

CtrlZ 的状态链路：

1. `types.hpp` 的 `IMU_ID_MAP=29`，对应 bus 中的 `imu`。
2. 模型中的 `imu` site 位于 pelvis，site 相对 pelvis 无旋转；`secondary_imu` 只记录另一组姿态，
   没有进入该策略的 `AngleValue` / `AngleVelocityValue`。
3. `MujocoImu::Input` 从 `imu_gyro` 读取 site 坐标系角速度，从 `site_xmat` 计算 RPY。
4. `ImuProcessWorker` 滤波权重都是 1，不改变信号；单位是 rad、rad/s。显示监控才转为度。
5. `ComputeProjectedGravity` 执行 RPY → XYZW 四元数 → 逆旋转 `(0,0,-1)`。

本框架的 CA PE 策略从同步 UDP 包读取 pelvis 的角速度和重力，不使用 DDS 中的 IMU 字段做推理。
原实现使用：

```cpp
mj_objectVelocity(model, data, mjOBJ_BODY, pelvis, velocity, 1);
```

MuJoCo `engine_support.c::mj_objectVelocity` 对 `mjOBJ_BODY` 使用 `ximat`，即机体惯性主轴系；
而 `mjOBJ_XBODY` 使用 `xmat`，即正常机体坐标系。原重力投影一直使用 `xmat`，因此两项观测混用了坐标系。
pelvis 的 inertial quaternion 为 `(1,0,-0.000399148,0)`，约有 0.046° 旋转，误差虽小仍应修正。

已将正式 C++ 观测改为 `mjOBJ_XBODY`，同步修正 Python 旧跑酷参考实现和闭环测试速度遥测。
正式 `unitree_mujoco` 已重新构建；需要重启仿真器才能加载修正。

数值验证使用真实 C++ UDP 发布/解码、MuJoCo gyro sensor、实际策略的 Observe，并直接编译调用
CtrlZ `Utils/so3.hpp` 中的数学模板。覆盖 45 组 roll/pitch/yaw 组合，以及独立的腰部运动：

| 检查 | 最大绝对误差 |
| --- | ---: |
| 修正前 BODY 角速度 vs pelvis gyro，测试角速度量级 ≤0.2 rad/s | 1.597×10⁻⁴ rad/s |
| 修正后 UDP 角速度 vs pelvis gyro | 7.18×10⁻⁹ rad/s |
| UDP 重力 vs CtrlZ RPY→四元数逆旋转 | 1.79×10⁻⁷ |
| 策略当前观测前 6 维（角速度×0.2 与重力） | 1.79×10⁻⁷ |

测试还确认 pelvis 与 torso 重力差最大约 0.286，避免测试因两者姿态相同而漏掉选错 IMU 的问题。
这证明了同一 MuJoCo 状态下的数学/采集对齐，不代表两个完整运行系统的时序和轨迹已经一致。

## 投影重力：未发现符号或轴序错误

本框架使用 `[-R20,-R21,-R22]`，等价于 `Rᵀ·(0,0,-1)`；CtrlZ 的函数也是同一逆旋转。
roll、pitch、yaw 顺序和四元数内部 XYZW 顺序都已核对。正常直立应为 `(0,0,-1)`；
单独改变绝对 yaw 不改变重力向量，因此无需减初始 yaw，也不应为了压住右偏反转 gyro_z 或重力分量。
修正后的角速度使用与重力一致的 pelvis 机体系，正 yaw 为向左转。

## 其他已查明的差异

| 项目 | CtrlZ 当前配置/源码 | 本框架当前实现 |
| --- | --- | --- |
| 策略周期 | scheduler 2 ms × InferTask 分频 10 = 20 ms；这里的 10 不是 10 Hz | 20 ms |
| 物理 | 2 ms，场景指定 RK4 | g1_test JSON 5 ms，当前平地场景默认积分器 |
| 相机朝向 | yaw-only(torso) × offset，位置仍跟随完整 torso 姿态 | 完整 torso 姿态 × offset |
| 相机中心 | `(W-1)/2,(H-1)/2`，再用像素 `col+0.5,row+0.5` | `W/2,H/2`，像素 `col+0.5,row+0.5` |
| 深度命中 | group 0+1；最近命中不足 min_distance 时排除一个 body 重投一次 | 地面+视觉几何；按每个 geom 的射线距离过滤近命中 |
| 相机轴方向 | flip_horizontal/vertical 均为 true，生成 x前/y左/z上射线 | 同为 x前/y左/z上，不存在简单的左右图像反号差异 |
| 本体历史 | 整帧旧→新；最近动作；启动时环形缓冲先为零 | g1_test 整帧旧→新；最近动作；第一帧填充缓冲 |
| 深度历史 | 37 帧中取 offset 35,30,…,0 | 同样 37 帧取 35,30,…,0 |
| 观测裁剪 | 本体 ±100，深度 [0,1] | 本体不额外裁剪，深度 [0,1] |
| 关节映射/默认角/增益/动作缩放 | 映射与当前 CA PE 一致，参数保留约六位小数 | 一致，JSON 保存更多有效位 |
| 机器人版本 | g1_29dof_rev_1_0.xml | g1_29dof.xml |
| XML 显式惯性质量之和 | 33.340142 kg | 35.112142 kg |
| torso 质量 | 7.817 kg | 9.598 kg |
| 脚底 | 连续盒体 | 四个球形接触点 |
| hip roll 力矩上限 | 139 Nm | 正式 MJCF 上限 88 Nm |
| 阻尼 | XML 被动 joint damping=训练 Kd，设备 position PD 另有 `-pos_kd*dq` | 视觉通道将被动 damping 清零，仅显式 PD 中的 Kd |

CtrlZ 的 JSON `MotorPDLoop` 并不是唯一控制来源：`user_func.cpp` 中额外 MotorPDWorker 创建被注释，
实际 position 模式使用 bus XML 的 `pos_kp/pos_kd`；`MujocoJoint::Output` 计算 PD，
模型中还有被动 damping。因此两边即使 JSON 的 Kd 数字相同，总阻尼仍不同。

以上差异是源码和资源核对结果，尚未单独完成每项的同模型闭环消融，不能把某一项直接命名为右偏根因。
尤其相机 YAML 写了 yaw 并不代表训练自定义传感器一定执行 yaw-only，需要结合其函数实现和对应训练版本判断。
本次没有为了匹配 CtrlZ 而更换正式模型、增加阻尼、改相机朝向或改物理周期。

## 修正后的闭环复测

使用 g1_test 原模型、原 JSON 和原场景，除角速度坐标修正外不改变配置：

- 零速落地约 8 秒：最低 pelvis 0.760 m，最大水平漂移 0.032 m。
- 逐步给定到 `(0.5,0,0)`：4.5～9 秒平均速度 `(0.498,-0.009)` m/s，
  yaw 角速度约 `-0.187 rad/s`；修正前同一测试约 `-0.190 rad/s`。

因此 IMU 坐标错误已确认且已修正，**但在当前 g1_test 测试中没有消除持续右转**。
该数据为单次检查，不作改善幅度的统计推断，也不代表已证明策略训练存在问题。

本次相关检查共 6 项通过：g1_test 离线推理、IMU/重力数值对照、新策略落地站立与前进、
旧 parkour 离线推理与落地站立。旧策略数值参考的 Python 实现已同步改为 XBODY，
本次没有运行该 Python 路径的独立数值回归。正式构建与临时产物清理检查完成。

数值复测入口：

```bash
python3 example/cpp/tests/run_tests.py \
  --test ca-pe-observation --task-dir example/exported/g1_test \
  --ctrlz-root '/home/mayh/MYH/Project_code/C++/CtrlZ' --verbose
```

不提供 `--ctrlz-root` 时，使用等价 Euler 公式作检查，无需 CtrlZ 依赖。
该检查使用本机 UDP，不发 DDS 电机目标，不启动运动；构建及产物均由统一入口自动清理。
