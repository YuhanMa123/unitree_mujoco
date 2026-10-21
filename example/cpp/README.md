# G1 C++ 推理与控制

以下命令均在仓库根目录执行。关节初始化集成在控制器内，不单独生成复位程序。

| 程序 | 用途 | 配套资源 |
| --- | --- | --- |
| `g1_locomotion` | G1 29DOF 盲走；通常连接 C++ 仿真器 | [YAML 与模型](../exported/g1_locomotion/) |
| `g1_parkour` | 跑酷双网络推理；连接 C++ 仿真器的同步状态和深度 | [JSON 与模型](../exported/g1_parkour/) |
| `g1_ca_pe_amp_vae` | pelvis 状态 + 深度 token + VAE/交叉注意力视觉运控 | [JSON、模型与运行步骤](../exported/g1_CA_PE_AMP_VAE/README.md) |
| `gamepad_monitor` | 只读手柄诊断 | `simulate/gamepad.yaml` |

## 编译

依赖 C++17、CMake、Unitree SDK2、Boost、yaml-cpp 和 ONNX Runtime C/C++ 发行包。
仿真器安装与构建见[项目说明](../../readme_zh.md)。

本机已有依赖路径缓存，可直接执行：

```bash
cmake -S example/cpp -B example/cpp/build \
  -DBUILD_G1_LOCOMOTION=ON -DBUILD_G1_PARKOUR=ON
cmake --build example/cpp/build --target g1_locomotion g1_parkour gamepad_monitor -j4
```

新机器首次配置时，在上述 `cmake -S` 命令后补充实际依赖路径：

```text
-DCMAKE_PREFIX_PATH=/path/to/unitree_robotics/lib/cmake
-DONNXRUNTIME_ROOT=/path/to/onnxruntime
```

ORT 目录应包含 `include/onnxruntime_cxx_api.h` 和 `lib/`，头文件与动态库必须来自同一版本。
运行时保留 SDK/DDS 与 ORT 动态库；搬迁仓库后重新配置编译，或显式指定模型和配置路径。

## G1 locomotion

终端 A 启动 C++ 仿真器，使用 G1 29DOF 平地场景、DDS domain 1 和回环网卡：

```bash
./simulate/build/unitree_mujoco -r g1 -s scene_29dof_flat.xml -i 1 -n lo
```

`simulate/config.yaml` 中应开启 `enable_elastic_band: 1`。终端 B 启动控制器：

```bash
./example/cpp/build/g1_locomotion

# 可选位置参数：复位秒数、网卡、DDS domain、部署配置、模型
./example/cpp/build/g1_locomotion 3 lo 1 \
  example/exported/g1_locomotion/g1_locomotion.yaml \
  example/exported/g1_locomotion/policy.onnx
```

1. 控制终端按 `9` 使能，再按 `p` 复位，等待 `Holding default pose`。
2. MuJoCo 窗口按 `8` 放长挂带，使双脚接地。
3. 控制终端按 `r` 启动策略；MuJoCo 窗口按 `9` 释放挂带。
4. 用下表按键输入速度，`x` 停止行走。手柄流程见 [Xbox 操作](../../doc/g1_xbox_cpp.md)。

| 控制终端按键 | 行为 |
| --- | --- |
| `9` | 使能命令输出；首次开始关节复位后锁定 |
| `p` | 从实测角度平滑复位并保持；策略运行时退出策略并复位 |
| `r` | 复位完成且各关节距默认角不超过 0.35 rad 时启动策略 |
| `w/s`、`a/d`、`q/e` | 分别增减前向、横向、转向命令，每次 0.1 |
| `x` | 速度清零，策略继续运行 |
| 空格 / Ctrl+C | 撤去仿真控制并退出 |

速度按键累加并保持，范围由 YAML 限制。控制终端的 `9` 与 MuJoCo 窗口的挂带 `9` 按焦点区分。
RB 或仿真窗口 Backspace 执行整机仿真重置：保留策略是否运行，清空历史和速度命令后继续。
B / `p` 仅做关节初始化。复位期间挂带提供支撑，关节保持本身不负责浮动基座平衡。
仿真窗口 `7` / 上方向键缩短挂带、吊起，`8` / 下方向键放长挂带、放下。
挂带只施加平移力，不锁定朝向；静态检查应保持关节初始化模式。

默认推理 50 Hz、控制发布 500 Hz。部署 YAML 管理姿态、PD、关节映射、观测和动作参数；
修改后重启控制器即可。模型接口与校验值见[模型说明](../exported/g1_locomotion/README.md)。

## G1 parkour

C++ 仿真器负责物理、torso 状态与深度采集，`g1_parkour` 通过 ONNX Runtime CPU 执行双网络推理。
观测由本机 UDP `17200` 发送，控制通过 DDS `rt/lowcmd` 返回；这条链路不需要 Python SDK、Torch 或 Isaac Sim。

```bash
# 编译 C++ 仿真器（依赖安装与初次配置见项目说明）
cmake --build simulate/build --target unitree_mujoco -j4

# 终端 A：先在平地验证站立，必须添加 --parkour
./simulate/build/unitree_mujoco -r g1 -s scene_29dof_flat.xml -i 1 -n lo --parkour

# 终端 B：运行 C++ 推理
./example/cpp/build/g1_parkour
```

按 `9 → p`，等待 `Holding training pose`，在仿真窗口放长挂带使脚接地，
回到控制终端按 `r` 启动策略，再在仿真窗口按 `9` 释放挂带。
控制终端 `w/s` 调整前向速度、`q/e` 调整转向、`x` 清零；横向速度固定为 0。
`p` 返回复位，空格或 Ctrl+C 退出。

手柄也可直接控制跑酷：**B 自动使能并复位 → 等待 `Holding training pose` → Menu 启动策略**。
左摇杆前后或十字键上下控制前向速度，右摇杆控制转向；横移和倒退固定为 0。
View / R3 清零速度，B 返回关节复位；Y/A 只升降挂带，保持策略运行。
策略悬空运行时仍可能持续摆腿、扭腰。需要吊起检查保持姿态时先按 B / `p`，
待初始化完成后再升降；View / R3 只清零速度命令，不退出策略。
建议按住 A 缓慢降到双脚接地，再按 X 释放挂带；不要从高处直接释放。
RB 执行整机仿真重置。LB 切换机器人居中跟随：开启时切到背后，鼠标可自由旋转/缩放；
LT 按住后跟随朝向，松开后恢复自由角度，居中仍由 LB 决定。摇杆在启动、停止及重连后需回中。
控制器复用 `simulate/gamepad.yaml`，可用 `--no-gamepad` 临时禁用手柄。

直接按 `r` / Menu 不会跳过使能与关节复位；未就绪时会打印具体原因。
键盘输入需让控制终端获得焦点；MuJoCo 窗口的按键用于挂带等仿真操作。

默认配置为 `example/exported/g1_parkour/g1_torsobase_cfg.json`。需要覆盖路径或端口时，两端一致设置：

```bash
# 终端 A
./simulate/build/unitree_mujoco -r g1 -s scene_29dof_flat.xml -i 1 -n lo --parkour \
  --parkour-config example/exported/g1_parkour/g1_torsobase_cfg.json --parkour-port 17200
# 终端 B
./example/cpp/build/g1_parkour --domain 1 --port 17200 --reset-seconds 3 \
  --config example/exported/g1_parkour/g1_torsobase_cfg.json \
  --model-dir example/exported/g1_parkour
```

仿真物理步长为 2 ms，状态与深度同步采样、推理周期为 20 ms，DDS 目标发布 500 Hz。
跑酷仿真读取配置中的 armature。尚无电机控制命令时保留 MJCF 原有被动阻尼/摩擦，
避免悬空自由关节持续摆动；初始化/策略输出启用后去掉额外被动损耗，与参考推理一致。
仿真每步重新计算 PD 并按训练/模型较小力矩上限裁剪。相机使用 torso 姿态，通过 CPU 射线输出
36×64 图像平面深度；深度预处理和历史缓存仍在 C++ 推理端执行。

RB/Backspace 重置仿真会保留策略是否运行，跑酷关节恢复训练默认姿态，清空历史、上一动作及速度命令后使用新状态继续。
积压的观测逐帧处理，不丢弃中间历史。最多补齐 5 个缺失采样（间隔上限 120ms）：
本体状态插值，深度沿用最近实测图像，期间保持 PD 输出；单次推理超过 20ms 先记录延迟。
超过恢复窗口、连续 150ms 未收到观测、DDS 状态超时或姿态异常仍会保护停止；停止后需重启控制器。
切换模型或场景请重启两个程序；将 `-s` 改为 `scene_29dof.xml` 可加载仓库地形。
控制器每 2 秒打印处理帧数、躯干高度/倾角、归一化深度范围与推理耗时。
躯干低于 0.5m 或倾斜超过 57.3° 时停止输出，并打印触发时的实际数值。
挂带落地与地形前进有闭环检查；完整跨障能力与完整速度范围仍需逐项验收。

必要文件与网络接口见[部署资源说明](../exported/g1_parkour/README.md)。
[Python 参考实现](../parkour/README.md)用于离线对照；原 Python 仿真与观测发送路径也保留，
但不要与 C++ 仿真器在相同 domain/端口同时运行。

## G1 CA PE AMP VAE

新视觉任务使用独立 `g1_ca_pe_amp_vae`，复用跑酷控制流程，并按训练快照匹配 pelvis 状态、
5 帧整帧本体历史和 8 帧 32×32 深度 token。构建时添加 `-DBUILD_G1_CA_PE_AMP_VAE=ON`。
启动 C++ 仿真器时使用 `--parkour`，并通过 `--parkour-config` 指定新任务 JSON；不能沿用旧跑酷默认配置。
完整命令、模型接口与参数说明见 [新视觉部署说明](../exported/g1_CA_PE_AMP_VAE/README.md)。
项目各模块关系见 [项目结构](../../doc/project_structure.md)。

## 实机部署

仅 `g1_locomotion` 提供实机入口，尚未实机验收。请在机载电脑按其架构重新构建，
核对机器人修订版、关节边界、训练机体系及模型参数，并使用吊架和独立停止手段。
仿真默认使用正数 DDS domain；实机必须显式使用 `--hardware`，domain 为 0，网卡不能是 `lo`。

先用 `ip -br addr` 确认连接运动控制板的网卡，将实际名称填入 `ROBOT_IFACE`，再执行只读核验：

```bash
./example/cpp/build/g1_locomotion --hardware-read-only 3 "$ROBOT_IFACE" 0 \
  example/exported/g1_locomotion/g1_locomotion.yaml
```

只读模式校验 LowState 的 CRC、模式、电机及 IMU，不加载策略、不释放运动服务、不下发控制。
核验模型与硬件对应关系后，控制入口为：

```bash
./example/cpp/build/g1_locomotion --hardware 3 "$ROBOT_IFACE" 0 \
  example/exported/g1_locomotion/g1_locomotion.yaml \
  example/exported/g1_locomotion/policy.onnx
```

操作为 `9 → p → 等待复位完成 → r`。`9` 通过 MotionSwitcher 接管服务并使能输出，不操作物理电源。
实机模式使用键盘；不启用原生 Xbox 仿真控制，也没有虚拟挂带。退出不会自动恢复官方运动模式。

`hardware` 配置段控制状态超时、姿态、温度、关节边界和目标变化率等限制，阈值属于项目设置。
停止时尝试发布约 0.5 秒阻尼命令；遥控器 A 也触发软件停止，但不代表固件急停。
该逻辑不构成硬实时看门狗或安全认证；进程卡死、断电、网络失联时不能保证软件停止命令执行，
PD 力矩估计也不等于硬件限幅或完整的 PR/AB 耦合力矩模型。

接口依据：[官方 SDK 示例](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/g1/low_level/g1_ankle_swing_example.cpp)、
[遥控器布局](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/g1/low_level/gamepad.hpp)、
[G1 rev_1_0 URDF](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/g1_description/g1_29dof_rev_1_0.urdf)。

## 验证与监控

离线检查不启动 DDS、不下发命令；测试在临时目录构建，结束后自动删除产物：

```bash
./example/cpp/build/g1_locomotion --check-policy
./example/cpp/build/g1_parkour --check-policy
python3 example/cpp/tests/run_tests.py
```

按需检查和依赖设置见[测试说明](tests/README.md)。盲走集成脚本需要 Python 仿真依赖；跑酷 C++ 集成检查只需现有 C++ 依赖。
检查会启动自己的仿真和正式控制器，使用独立 domain：盲走 191，跑酷 195..202。

```bash
python3 example/cpp/tests/g1_locomotion_mujoco_smoke.py --ground
# C++ 仿真/UDP/双网络/DDS/PD：固定基座重入、落地站立、超时停止、整机重置后继续推理
python3 example/cpp/tests/run_tests.py --test parkour-sim
```

网页关节与 IMU 监控使用同一个 DDS domain / 网卡，C++ 工具只订阅状态、命令、目标和使能：

```bash
cmake -S simulate -B simulate/build
cmake --build simulate/build --target joint_monitor -j4
./simulate/build/joint_monitor --domain 1 --interface lo --port 8765
```

浏览器打开 `http://127.0.0.1:8765`。Enable 表示控制器的命令输出状态，不是物理上电反馈。
支持多关节曲线、搜索、角度单位切换和 IMU 显示；无需 Python，
详见 [C++ 网页监控](../../simulate/README_joint_monitor.md)。Python 兼容入口仍为 `simulate_python/joint_monitor.py`。
手柄只读检查可运行 `./example/cpp/build/gamepad_monitor --once`。

同一 DDS domain 中只运行一个发布 `rt/lowcmd` 的控制器。短时仿真或离线检查通过，
不代表已验证长期稳定性、完整速度范围或实机运行。
