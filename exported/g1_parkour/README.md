# G1 Parkour ONNX 模型

C++部署和Python参考推理默认从此目录加载两个网络，不再依赖训练目录。
部署步骤见 [Python MuJoCo + C++ 推理](../../example/cpp/README_g1_parkour.md)。
对应关节、方向和PD参数为 [`example/parkour/g1.json`](../../example/parkour/g1.json)。

| 文件 | 输入 | 输出 |
| --- | --- | --- |
| `0-depth_encoder.onnx` | float32 `[1,8,18,32]` | float32 `[1,128]` |
| `actor.onnx` | float32 `[1,896]` | float32 `[1,29]` |

动作网络输入为768维本体历史与128维深度编码特征；两个模型需配套使用。
仓库根目录的 `policy.onnx` 属于另一个locomotion控制器。

原始导出目录：

```text
/home/wsy/rl/InstinctLab/logs/instinct_rl/g1_parkour/20260905_091435/exported
```

SHA256（与原文件一致）：

```text
b184217a93d3e54a2cf79b786be71f296bccea061e20cfcd454d9877f0633b8e  0-depth_encoder.onnx
24119b88e71394bbfb0cfa68479bded782d6fe016f132f79b3d3d3b61880343b  actor.onnx
```
