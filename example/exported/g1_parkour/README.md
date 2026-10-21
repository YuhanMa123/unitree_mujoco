# G1 Parkour ONNX 模型

C++部署和Python参考推理默认从此目录加载两个网络，不再依赖训练目录。
部署步骤见 [C++ MuJoCo + C++ 推理](../../cpp/README.md#g1-parkour)。
对应关节、方向和PD参数为 [`g1_torsobase_cfg.json`](g1_torsobase_cfg.json)。

| 文件 | 输入 | 输出 |
| --- | --- | --- |
| `0-depth_encoder.onnx` | float32 `[1,8,18,32]` | float32 `[1,128]` |
| `actor.onnx` | float32 `[1,896]` | float32 `[1,29]` |

动作网络输入为768维本体历史与128维深度编码特征；两个模型需配套使用。
`../g1_locomotion/policy.onnx` 属于另一个locomotion控制器。

原始导出目录：

```text
/home/wsy/rl/InstinctLab/logs/instinct_rl/g1_parkour/20260905_091435/exported
```

SHA256（与原文件一致）：

```text
b184217a93d3e54a2cf79b786be71f296bccea061e20cfcd454d9877f0633b8e  0-depth_encoder.onnx
24119b88e71394bbfb0cfa68479bded782d6fe016f132f79b3d3d3b61880343b  actor.onnx
```

## 运行所需文件

| 文件 | 职责 |
| --- | --- |
| 本目录的两个 ONNX 与 `g1_torsobase_cfg.json` | 模型、关节顺序与方向、姿态、PD、动作缩放及力矩限制 |
| `example/cpp/g1_parkour_policy.h` | 深度预处理、8 帧本体历史、深度抽帧、双网络推理与动作映射 |
| `example/cpp/g1_parkour.cpp` | UDP 接收、使能/复位/推理状态机、DDS 发布与超时保护 |
| `example/cpp/g1_hardware.h` | 复用 DDS 命令 CRC 计算；此任务仍为仿真入口 |
| `simulate/src/parkour_observation.h` | C++ 训练电机参数、同步状态与深度采样、UDP 打包、复位姿态和力矩限幅 |
| `simulate/src/main.cc`、`param.h`、`unitree_sdk2_bridge.h` | 仿真入口、`--parkour` 参数、每步 PD 与 DDS 桥接 |
| `unitree_robots/g1/` | 场景、机器人 MJCF 与 meshes |

两个网络及部署参数必须配套使用。配置由训练源码重建，缺少训练时参数快照；
训练 torso 根模型与当前 pelvis 根 MuJoCo 模型存在动力学差异，腰关节方向转换已在部署配置中声明。
