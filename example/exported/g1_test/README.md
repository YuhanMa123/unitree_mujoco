# g1_test 仿真推理

本任务复用 `g1_ca_pe_amp_vae`，不需要新增控制器。运行资源均在本目录：
`params/` 为训练参数，`exported/actor.onnx` 和 `exported/depth_token_encoder.onnx`
为 C++ 使用的拆分模型，`g1_ca_pe_amp_vae_cfg.json` 为本任务专用部署参数。
`exported/policy.onnx` 是完整网络，`policy.pt` 不由当前 C++ 入口加载。

## 适配核对（2026-10-20）

| 项目 | 结果 |
| --- | --- |
| 机器人 / 关节 | G1 29DOF、pelvis 状态；顺序、方向、默认角、PD 与动作 scale 和旧 CA PE 任务相同 |
| 周期 | 物理 5 ms，策略 20 ms |
| 当前观测 / 历史 | 96 维，5×96 整帧历史，从旧到新并包含当前帧 |
| 动作历史 | 训练归档 `git/robot_lab.diff` 明确将历史改为 `action_manager.action`；配置 `history_action_delay: 0` |
| 深度 | torso 相机，36×64 裁剪至 32×32，8 帧、步长 5，归一化至 [0,1] |
| ONNX | depth `[1,8,32,32]` → tokens `[1,64,64]`；actor `(96,480,tokens)` → 29 动作 |
| 新注意力查询 | 增加 16 维 VAE latent，已封装在 ONNX 中，无需 C++ 重写 |
| 速度范围 | x [-0.5,1.5]、y [-0.5,0.5] m/s；yaw [-0.5,0.5] rad/s |

原目录缺少部署 JSON；现已根据本任务快照生成。控制器新增可配置动作历史时序：
本任务使用 0，缺少该字段的旧任务仍默认 1，保持原行为。
参数提取器未自动从 YAML 推断时序，因为 YAML 只记录函数引用；此处依据随训练保存的源码差异。

训练新增的地形、奖励、目标导航参数不需要直接传入手动速度控制入口。
运行时不复现训练目标点生成器、随机化及深度随机噪声，也不自动生成训练同款地形。
MuJoCo 仍使用现有脚底接触模型；hip roll 的实际力矩上限受 MJCF 限制为 88 Nm，
训练 JSON 为 139 Nm。此部署验证不代表与训练物理环境完全等价。

已验证：正式仿真器/控制器构建、参数一致性、真实双网络推理、动作与历史、视觉/命令响应、
完整 ONNX 与拆分 ONNX 的合成输入输出对照，以及真实 C++ MuJoCo/UDP/ONNX/DDS/PD 闭环。
旧 CA PE 策略的离线回归检查也通过。

- 零命令落地约 8 秒：最低 pelvis 0.759 m，最大水平漂移 0.038 m。
- 前进命令逐步增加至 0.5 m/s：最低 pelvis 0.726 m；4.5～9 秒平均机体系速度
  `(0.497,-0.011)` m/s，平均 yaw 角速度 `-0.190` rad/s；t≈9 秒 yaw 约 -53°。
- 以上为单次受控测试；闭环检查通过标准覆盖站立/前进和不倒地，**不代表直线跟踪合格**。
  新模型仍有持续右转，尚未验证 1 m/s、长期运行或完整跨障能力。

后续 [CtrlZ 对齐检查](../../../doc/g1_ctrlz_alignment.md) 修正了角速度误用惯性主轴系的问题，
并通过真实 pelvis IMU 与 CtrlZ 重力数学实现的 45 组数值对照。修正后前进平均 yaw 角速度
仍约 -0.187 rad/s，未消除右转；正式仿真器已重新构建。

行走后仰的参数核对与对照实验见[后仰排查](../../../doc/g1_ca_pe_backward_lean.md)：
默认角、映射和参数快照一致，但两框架的有效阻尼、相机姿态和机器人模型存在差异。
测试通过不表示已解决后仰。相机可选 `camera.ray_alignment: body | yaw`，默认保持 body；
当前正式配置未切换到 CtrlZ 的 yaw 模式。

## 完整启动命令

以下均在仓库根目录运行。两端必须使用本任务 JSON，控制器也必须显式指定本任务模型目录。

```bash
cmake -S example/cpp -B example/cpp/build -DBUILD_G1_CA_PE_AMP_VAE=ON
cmake --build example/cpp/build --target g1_ca_pe_amp_vae -j4
cmake -S simulate -B simulate/build
cmake --build simulate/build --target unitree_mujoco -j4

# 仅加载模型并推理，不启动 DDS 控制
./example/cpp/build/g1_ca_pe_amp_vae \
  --config example/exported/g1_test/g1_ca_pe_amp_vae_cfg.json \
  --model-dir example/exported/g1_test \
  --check-policy
```

终端 A，启动仿真器：

```bash
./simulate/build/unitree_mujoco \
  -r g1 -s scene_29dof_flat.xml -i 1 -n lo \
  --parkour \
  --parkour-config example/exported/g1_test/g1_ca_pe_amp_vae_cfg.json \
  --parkour-port 17200
```

终端 B，启动控制器：

```bash
./example/cpp/build/g1_ca_pe_amp_vae \
  --config example/exported/g1_test/g1_ca_pe_amp_vae_cfg.json \
  --model-dir example/exported/g1_test \
  --domain 1 \
  --port 17200 \
  --reset-seconds 3
```

使用键盘时可在控制器命令末尾加 `--no-gamepad`，避免手柄同时输入。
本机同一 domain/端口只运行一套仿真器和运动控制器；切换任务需重启两端。

操作步骤：

1. 控制终端按 `9`、`p`，等待 `Holding training pose`。
2. MuJoCo 窗口按 `8` / 下方向键放长挂带，直到双脚接地。
3. 控制终端按 `r` 启动策略；MuJoCo 窗口按 `9` 释放挂带。
4. 控制终端 `w/s` 调整前后、`a/d` 调整横向、`q/e` 调整转向，每次 0.1；`x` 清零，
   `p` 返回关节初始化，空格或 Ctrl+C 退出。

手柄：`B → 等待初始化 → A 放低到接地 → Menu 启动 → X 释放挂带`。
左摇杆控制前后/横向，右摇杆转向，View / R3 清零。
共享手柄前向上限为 1.0 m/s，键盘前向上限由任务 JSON 限定为 1.5 m/s。
这些是指令上限，不是已验证的实际稳定行走速度。

## 配置维护与复测

```bash
python3 example/cpp/tools/export_ca_pe_config.py \
  --task-dir example/exported/g1_test --history-action-delay 0
python3 example/cpp/tools/export_ca_pe_config.py \
  --task-dir example/exported/g1_test --check
python3 example/cpp/tests/run_tests.py \
  --test ca-pe --task-dir example/exported/g1_test --verbose
python3 example/cpp/tests/run_tests.py \
  --test ca-pe-sim --task-dir example/exported/g1_test \
  --sim-mode ground --sim-mode walk --verbose
```

`--task-dir` 同时指定检查的 JSON 和实际模型目录，避免使用新 JSON 却误测旧模型。
测试构建位于独立临时目录，结束后自动清理。

模型 SHA256：

```text
f48e18740e2ad1e2e85af6bba0f2f4174460a88cb324be874873c5f0200b29d8  exported/actor.onnx
e65bfbf83152670ea07feaf1bbc87aac464caf712b87be1e88ca7793e8825a3c  exported/depth_token_encoder.onnx
7eb343b957841cd1334557ad037fa26c35d568e8bb71cbc7ccce3096a1fb27ec  exported/policy.onnx
```
