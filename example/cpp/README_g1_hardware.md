# G1 29DOF 解锁腰部：机载部署入口

代码按官方 G1 SDK 示例接入实机通信，尚未在实机验收。
仿真仍使用正数 DDS domain；只有显式传入 `--hardware` 或
`--hardware-read-only` 才允许 domain 0。网卡必须明确指定，禁止 `lo`。

参考来源：

- [官方 G1 开发文档入口](https://support.unitree.com/home/en/G1_developer)
- [官方 SDK2 G1 低层控制示例](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/g1/low_level/g1_ankle_swing_example.cpp)：domain 0、MotionSwitcher、PR 坐标、CRC 和电机状态。
- [官方遥控器数据布局](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/g1/low_level/gamepad.hpp)：无线遥控按钮字节定义。
- [官方 G1 rev_1_0 URDF](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/g1_description/g1_29dof_rev_1_0.urdf)：逐关节位置、速度及 effort 边界。

## 在机器人电脑构建

在实际运行程序的机载电脑安装 Unitree SDK2、Boost、yaml-cpp、CMake、C++17
编译器及 CPU ONNX Runtime C/C++ 发行包。用 `uname -m` 判断架构，SDK 和 ORT
必须匹配该机载电脑；不能把当前开发机编出的 x86_64 可执行文件直接复制给 ARM。

将完整项目（包括 `policy.onnx` 和 YAML）复制到机器人电脑，在项目根目录执行。
先将 `ONNXRUNTIME_ROOT` 设置为该机上发行包目录，里面应有 `include/` 与 `lib/`：

```bash
cmake -S example/cpp -B example/cpp/build \
  -DBUILD_G1_LOCOMOTION=ON \
  -DONNXRUNTIME_ROOT="$ONNXRUNTIME_ROOT"
cmake --build example/cpp/build --target g1_locomotion g1_hardware_test g1_locomotion_policy_test -j4
ctest --test-dir example/cpp/build --output-on-failure
./example/cpp/build/g1_locomotion --check-policy
```

这些检查不连接机器人。保留 ORT 发行包及 SDK 运行库；当前默认路径指向构建时
项目源目录。移动部署文件后，应在命令中明确传入 YAML 和模型路径。

## 先只读核验

用 `ip -br addr` 找到实际连接机器人运动控制板的网卡。机载部署不代表网卡为
`lo`，也不假定网卡一定叫 `eth0`。下例先填写 `ROBOT_IFACE`，之后执行：

```bash
./example/cpp/build/g1_locomotion --hardware-read-only 3 "$ROBOT_IFACE" 0 \
  example/cpp/g1_locomotion.yaml
```

只读模式只订阅 `rt/lowstate`，不释放原运动控制服务，不初始化 LowCmd 发布器，
也不加载策略模型。最多等待 5 秒，收到通过 CRC、型号、电机及 IMU 检查的状态
后返回。失败时报告原因；该检查验证当前状态，不等于已经验证模型能稳定行走。

## 控制入口

首次测试使用吊架和独立停止手段，保持人员远离运动范围。实机没有 MuJoCo
虚拟挂带；不能使用仿真中的 `8/9` 挂带操作。状态和模型对应关系核验后启动：

```bash
./example/cpp/build/g1_locomotion --hardware 3 "$ROBOT_IFACE" 0 \
  example/cpp/g1_locomotion.yaml policy.onnx
```

操作仍为 `9 → p → 等待复位完成 → r`：

- 启动后等待有效状态，暂不改变原有运动控制服务。
- 按 `9`：通过 MotionSwitcher 检查、释放原服务并确认释放；随后观察是否仍有
  其他 LowCmd 发布者。如果失败，拒绝进入控制。接管后、复位前发布零 Kp 的阻尼命令。
- 按 `p`：从实测位置平滑复位，复位姿态和 PD 统一读取 locomotion YAML。
- 按 `r`：复位完成且接近默认姿态后进入策略。推理中按 `p` 可返回复位。
- `W/S` 前后、`A/D` 横移、`Q/E` 转向、`X` 清零速度。速度按键累加并保持，沿用仿真操作。
- 空格、Ctrl+C、SIGHUP/SIGTERM 或低层状态中的遥控器 A 按钮触发停止。
  遥控 A 是本项目的软件阻尼停止映射，依据官方字节布局解析，**不是对固件急停按键的声明**。
- 首次进入复位后 `9` 锁定，需重启程序才能重新接管。

这里的“上电”是软件接管及命令输出使能，不操作机器人电源。
程序不自动恢复之前的官方运动模式，退出后的恢复按该机器人官方流程处理。

## 实机模式保护与限制

`g1_locomotion.yaml` 的 `hardware` 段仅实机模式启用：

- 严格要求 `mode_machine=5`，采用 PR 关节坐标；校验 LowState CRC。
- 检查全部 29 个关节的电机状态码、温度、位置、速度，以及 IMU 四元数和角速度。
- 默认状态超时 100 ms、倾角上限 1 rad、温度阈值 70°C；这些是项目初始设置，
  **不是官方推荐阈值或安全认证值**。任何状态验证失败会锁定故障，不能靠后续好帧自动恢复。
- 关节目标在 WalkNet1 处理后再次经过逐关节位置边界、0.01 rad 边界余量和目标变化率限制。
  默认目标变化率上限 10 rad/s，并受 URDF velocity 限制。
- 用当前 q/dq 和 PD 增益估计指令力矩，超过 URDF effort 时停止。
  该估计不等于电机驱动层的硬件力矩限幅，也未建立 PR/AB 耦合电机力矩模型。
- 推理返回后检查是否超过策略周期；控制和推理仍在同一线程中，不构成硬实时或
  独立看门狗。模型卡死、进程被 SIGKILL、断电或网络故障无法保证执行软件退出逻辑。
- 已接管并开始发布后，停止路径发送 mode=1、Kp=0、tau=0、dq=0、Kd=1
  的阻尼命令约 0.5 秒，然后结束进程；发送成功不代表控制板已确认执行。
  网络失联时需依赖已验证的控制板保护和独立停止手段。

逐关节边界来自官方 rev_1_0 URDF，使用前必须与实际硬件修订版核对。
PD、模型和观测历史仍按当前 WalkNet1 配置，不能因接口相同就认为已经完成
sim2real。实机策略使用 LowState 主 IMU（官方示例标为 pelvis），模型训练时的
机体系、传感器位置及关节顺序需与此相符。没有增加未经确认的 IMU 固定旋转。

当前验证范围：本机编译、离线保护逻辑测试、ONNX 推理测试和 MuJoCo 回归。
未连接实机，未验证实际 MotionSwitcher 响应、遥控器停止、机载运行时延或行走稳定性。
