# G1 CA PE AMP VAE 视觉运控部署

本目录保存训练参数快照、配套 ONNX 和提取后的部署参数。主运行链路为 C++ MuJoCo +
`g1_ca_pe_amp_vae`，运行时不需要 Python、Torch、IsaacLab 或训练仓库。
项目结构见 [项目与视觉部署结构](../../../doc/project_structure.md)。

| 文件 | 用途 |
| --- | --- |
| `params/env.yaml` | 训练环境快照：机器人、PD、观测、动作、相机与命令范围 |
| `params/agent.yaml` | 训练算法快照：模型结构、5 帧本体历史、动作裁剪 |
| `g1_ca_pe_amp_vae_cfg.json` | C++ 仿真器与控制器共同读取的部署参数 |
| `exported/depth_token_encoder.onnx` | 图像编码与位置编码，输出视觉 token |
| `exported/actor.onnx` | VAE 均值推理、状态估计、交叉注意力融合及动作输出 |

## 推理接口

| 网络 | 输入 float32 | 输出 float32 |
| --- | --- | --- |
| `depth_token_encoder.onnx` | `depth_image [1,8,32,32]` | `depth_tokens [1,64,64]` |
| `actor.onnx` | `actor_obs [1,96]`、`obs_history [1,480]`、`depth_tokens [1,64,64]` | `actions [1,29]` |

ONNX 导出的 batch 维是动态的，部署固定 batch=1；深度输出的 token 维在图声明中也是动态的，
运行时严格校验为 64×64。不能将两个模型按旧跑酷模型的 768+128 拼接方式调用。
归一化、VAE 和注意力融合以导出图为准，C++ 不额外采样 VAE 随机变量。

单帧 96 维依次为：角速度×0.2（3）、重力投影（3）、速度命令（3）、
相对默认关节位置（29）、关节速度×0.05（29）、上一次裁剪后的动作（29）。
5 帧本体历史按**整帧、从旧到新**拼接，与旧跑酷按观测项拼接的 8 帧历史不同。
注意训练实现的动作时序：当前观测使用 `action_manager.action`，历史帧使用
`ActorHistoryObservation._actor_frame` 中的 `action_manager.prev_action`，比当前观测的动作再旧一拍。
部署分别保存这两拍动作，不能直接把当前 96 维观测原样追加到历史。
该行为由 `history_action_delay` 控制，缺省为 1；使用最近动作构造历史的新训练任务应设为 0，
例如 [g1_test](../g1_test/README.md)，不能跨任务直接照搬。
启动与复位后用第一帧填充历史，上一动作清零。

深度为图像平面距离：torso 相机采集 36×64 → 裁掉顶部 4 行、底部 0 行、左右各 16 列 →
32×32 → 3×3 Gaussian blur（sigma=1，reflect padding）→ 裁剪至 [0,2.5] m → 除以 2.5。
保留 37 帧，每隔 5 帧取一帧，从旧到新输入 8 帧；`depth_delay_frames` 支持固定 0 或 1 帧延迟，默认 0。
射线按训练配置忽略各几何体上不超过 0.1 m 的最近命中，再在其余几何体中选最近距离；无命中深度保持远距离。训练的随机立体噪声、轮廓丢失和随机延迟不在确定性部署中重放。

机器人状态来自 **pelvis**，相机仍安装于 **torso_link**。
关节采用 IsaacLab pelvis 根的广度优先顺序，29 个方向均为 +1；不能复制旧跑酷的 torso 根关节顺序或腰关节反号。
动作先裁剪至 ±100，再按每关节 `default_joint_pos + action_scale × action` 生成目标，映射至 DDS。
仿真按训练参数设置 armature、物理步长 5 ms，每步重新计算 PD 并限制力矩；策略周期 20 ms。
未启用关节控制时保留模型原有被动阻尼/摩擦，初始化或策略输出启用后切换到训练动力学。
训练的命令生成器最终输出三维速度，因此此入口直接接收手动速度命令，不自动复现目标点采样与导航。

## 编译与运行

以下命令在仓库根目录执行；依赖设置见 [C++ 构建说明](../../cpp/README.md#编译)。

```bash
cmake -S example/cpp -B example/cpp/build -DBUILD_G1_CA_PE_AMP_VAE=ON
cmake --build example/cpp/build --target g1_ca_pe_amp_vae -j4
cmake -S simulate -B simulate/build
cmake --build simulate/build --target unitree_mujoco -j4

# 先检查实际模型加载与双网络推理，无 DDS 输出
./example/cpp/build/g1_ca_pe_amp_vae --check-policy
```

终端 A：

```bash
./simulate/build/unitree_mujoco -r g1 -s scene_29dof_flat.xml -i 1 -n lo \
  --parkour --parkour-config example/exported/g1_CA_PE_AMP_VAE/g1_ca_pe_amp_vae_cfg.json
```

`--parkour` 是现有同步视觉观测通道的开关；传入新 JSON 后，状态机体系、相机参数和物理步长均按新任务配置。
Python 旧跑酷观测发送器尚未适配本任务，应使用上述 C++ 仿真入口。

终端 B：

```bash
./example/cpp/build/g1_ca_pe_amp_vae

# 可覆盖路径；model-dir 指向本任务根目录，JSON 内的模型相对路径包含 exported/
./example/cpp/build/g1_ca_pe_amp_vae \
  --config example/exported/g1_CA_PE_AMP_VAE/g1_ca_pe_amp_vae_cfg.json \
  --model-dir example/exported/g1_CA_PE_AMP_VAE --domain 1 --port 17200
```

控制终端按 `9 → p → 等待 Holding training pose → r`。
MuJoCo 中先放长挂带到双脚接地，再释放挂带；手柄操作沿用 [C++ 操作说明](../../cpp/README.md#g1-parkour)。
`w/s` 前进/后退，`a/d` 横向，`q/e` 转向，`x` 速度清零，`p` 回到关节初始化，空格退出。
命令范围为 x∈[-0.5,1.5]、y∈[-0.5,0.5] m/s、yaw∈[-0.5,0.5] rad/s；手柄还受 `simulate/gamepad.yaml` 的范围限制。
手柄左摇杆与十字键的前向上限为 1.0 m/s；修改共享手柄配置后需重启仿真器和控制器。
该范围来自快照中的 `commands.base_velocity.ranges`，不自动切换训练中按地形设置的 `velocity_ranges`。

复用控制器的使能、平滑关节初始化、姿态保护、数据超时及丢帧恢复流程。
默认本机 UDP 17200，DDS domain 1、网卡 lo；同一 domain 只能运行一个运动控制器。
启动时会打印任务名称、JSON、模型目录和实际网络接口。

## 配置维护与检查

JSON 可从训练快照重新提取，无需加载训练框架。此开发工具需要 PyYAML：

```bash
python3 example/cpp/tools/export_ca_pe_config.py --check
python3 example/cpp/tools/export_ca_pe_config.py
python3 example/cpp/tests/run_tests.py --test ca-pe --test parkour
python3 example/cpp/tests/run_tests.py --test ca-pe-sim
```

`ca-pe` 验证关节映射、实际裁剪/模糊、历史顺序、真实双网络对视觉与命令的响应、动作反馈与清空历史。
`ca-pe-sim` 默认验证固定基座推理重入、落地站立、观测超时停机、仿真重置与落地重置后继续推理、丢帧恢复和平地前向行走；
它使用实际 C++ MuJoCo / UDP / ONNX / DDS / PD，测试构建位于临时目录并自动清理。
可用 `--sim-mode ground`、`--sim-mode jitter` 或 `--sim-mode walk` 选择单项检查。

相机近距离过滤修正后，已通过约 8 秒零命令落地站立（最低基座约 0.73 m，最大水平漂移约 0.042 m），
以及 80 ms 进程暂停、主动丢弃 5 个 UDP 包后的恢复检查。前向命令逐步增至 0.5 m/s 的平地检查也通过，
约 8 秒落地期间最大水平位移 2.41 m，最低基座约 0.70 m；随后速度清零并正常退出。
本次最终验证包含 5 项离线检查和 15 项新旧视觉链路检查，全部通过。结果不覆盖长期运动或完整跨障。

上述前进检查只验证位移与不倒地，不代表直线跟踪合格。2026-10-19 的专项检查复现了
零 yaw 指令下前进时持续右转，详见 [右偏排查与对照结果](../../../doc/g1_ca_pe_right_drift.md)。

MuJoCo 的机器人几何与训练 URDF/PhysX 存在差异；深度包含当前 MJCF 的机器人几何，
训练使用分组 visual meshes，遮挡细节仍可能不同。训练噪声、接触与执行器数值解也不完全相同。
固定基座链路检查不等同于完整跨障或实机验收；此入口面向本机仿真。

模型 SHA256：

```text
55cda3923bf5e38d382ce036277f3fcdc178f184c5414bcc6b7925c4d764b5d4  exported/actor.onnx
77d82df82cd9672bdf9e5b7355bd8d01afc284dcd487a4164676744db6a5a25c  exported/depth_token_encoder.onnx
```
