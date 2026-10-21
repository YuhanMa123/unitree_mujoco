# 推理部署资源

按任务保存配套的模型和部署参数；C++ 控制器、Python 参考推理、仿真观测采集和测试默认读取此目录。

| 任务 | 部署参数 | ONNX 模型 |
| --- | --- | --- |
| `g1_locomotion` | `g1_locomotion/g1_locomotion.yaml` | `g1_locomotion/policy.onnx` |
| `g1_parkour` | `g1_parkour/g1_torsobase_cfg.json` | `g1_parkour/0-depth_encoder.onnx`、`g1_parkour/actor.onnx` |
| `g1_CA_PE_AMP_VAE` | `g1_CA_PE_AMP_VAE/g1_ca_pe_amp_vae_cfg.json` | `g1_CA_PE_AMP_VAE/exported/depth_token_encoder.onnx`、`g1_CA_PE_AMP_VAE/exported/actor.onnx` |

新视觉任务的训练快照、网络接口与 C++ 运行步骤见 [CA PE AMP VAE 说明](g1_CA_PE_AMP_VAE/README.md)。

模型与参数必须配套更新。这里保存实际推理使用的部署参数，并非完整的训练参数快照。
跑酷配置由训练源码重建，来源及限制见 [跑酷说明](../parkour/README.md)。

旧的 `example/cpp/g1_locomotion.yaml` 副本已删除，盲走部署配置仅维护本目录中的文件。
历史文件 `policy.onnx`（仓库根目录）和 `example/parkour/g1.json` 暂时保留；默认入口不再使用这两个副本。

仿真器配置仍为 `simulate/config.yaml` 或 `simulate_python/config.py`，
手柄配置为 `simulate/gamepad.yaml`；机器人场景及网格位于 `unitree_robots/`。

从仓库根目录执行离线推理检查：

```bash
./example/cpp/build/g1_locomotion --check-policy
./example/cpp/build/g1_parkour --check-policy
```

编译时默认路径指向本目录的绝对路径。搬迁仓库后应重新配置、编译，或通过运行参数明确指定模型与配置。

G1 盲走启动与操作见 [C++ 盲走说明](../../doc/g1_blind_cpp.md)。

本次目录迁移已完成 C++ 仿真器及两类控制器构建、4 项 CTest 和两类控制器的默认路径离线推理检查；未运行图形界面行走测试。
