# 项目结构与视觉推理链路

项目包含仿真、部署控制、机器人资源和离线检查四部分。正式部署资源统一位于 `example/exported/<task>/`。

| 目录 | 职责与主要入口 |
| --- | --- |
| `simulate/` | C++ MuJoCo 仿真器；`src/main.cc` 运行物理与 UI，`unitree_sdk2_bridge.h` 收发 DDS 和执行 PD |
| `simulate/src/joint_monitor*` | 独立 C++ 只读 DDS 网页监控，编译为 `simulate/build/joint_monitor`；与 Python 版共享页面 |
| `simulate/src/parkour_observation.h` | 按任务 JSON 设置电机参数、复位姿态，采集同步状态及深度，经 UDP 发送，并限制力矩 |
| `simulate_python/` | Python 仿真及旧跑酷观测兼容实现；`joint_monitor.py` 只读关节/IMU 监控 |
| `example/cpp/` | 正式 C++ 控制器：`g1_locomotion` 盲走、`g1_parkour` 原跑酷、`g1_ca_pe_amp_vae` 新视觉策略 |
| `example/cpp/g1_visual_controller.h` | 两种视觉策略共用的 UDP/DDS 控制流程、使能、平滑初始化、运行与超时保护 |
| `example/cpp/g1_visual_observation.h` | 两种视觉策略共用的同步观测包、解码校验与丢帧补齐 |
| `example/cpp/g1_visual_config.h` | 视觉策略共用的关节映射、方向、默认姿态、动作缩放与 PD 参数解析 |
| `example/cpp/*_policy.h` | 各任务独立的观测排列、历史、ONNX 调用及动作处理 |
| `example/cpp/tests/` | 离线及完整 C++ 链路检查，统一由 `run_tests.py` 在临时目录构建和清理 |
| `example/cpp/tools/` | 训练快照到部署 JSON 的参数提取工具 |
| `example/exported/` | 按任务配套保存 JSON/YAML 与 ONNX；新视觉任务另保留 `params/` 训练快照 |
| `example/parkour/` | 旧跑酷 Python 参考推理与对照工具 |
| `example/ros2/` | ROS2 示例 |
| `include/unitree_mujoco/` | 共用手柄、仿真重置等头文件 |
| `unitree_robots/` | 各机器人 MJCF、场景与 meshes；G1 29DOF 场景在 `g1/` |
| `terrain_tool/` | 地形生成工具 |
| `doc/` | 构建、操作及部署说明 |

```mermaid
flowchart LR
  C[任务部署 JSON] --> S[C++ MuJoCo 仿真器]
  C --> P[C++ 策略适配层]
  S -->|UDP 同步状态与原始深度| P
  P --> D[深度 token ONNX]
  D --> A[Actor ONNX]
  P -->|当前观测与本体历史| A
  A --> T[动作缩放与关节映射]
  T -->|DDS 目标| S
```

三个任务不能混用模型和参数：

| 任务 | 状态与关节约定 | 网络输入与历史 |
| --- | --- | --- |
| `g1_locomotion` | 其 YAML 声明的盲走映射 | 单模型、本体观测 |
| `g1_parkour` | torso 根；腰关节方向转换 | 8 帧本体历史按观测项展开，8×18×32 深度编码为 128 维，再拼接为 896 维 actor 输入 |
| `g1_CA_PE_AMP_VAE` | pelvis 根；全部关节方向 +1，相机仍在 torso | 当前 96 维、5 帧×96 整帧历史、8×32×32 深度 → 64×64 token，actor 三输入 |

新任务的 `params/env.yaml` 定义机器人和输入输出语义，`params/agent.yaml` 定义 VAE 与交叉注意力架构。
部署仅提取观测、控制、相机与模型路径；奖励、AMP 专家动作、训练随机化及优化器参数无需进入运行配置。
VAE、位置编码与交叉注意力已由导出图实现，不需要在 C++ 重写。

构建及操作见 [C++ 说明](../example/cpp/README.md)，
新任务的配置、模型接口与运行命令见 [CA PE AMP VAE 部署说明](../example/exported/g1_CA_PE_AMP_VAE/README.md)。
