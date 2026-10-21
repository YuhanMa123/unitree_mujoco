# G1 DreamWaQ 盲走部署资源

`policy.onnx` 与 `g1_locomotion.yaml` 配套供 `g1_locomotion` 使用；
控制器内部的关节初始化也使用此处 YAML 中的默认姿态和 PD 参数。

模型 SHA256：`4f23975849ef66d73a170ee3435a274a4c42dd31830f8b07161add84846a871d`。

输入为当前观测 `[1,96]` 和五帧历史 `[1,480]`，输出为动作 `[1,29]`。
编译、运行及参数使用见 [控制器说明](../../cpp/README.md#g1-locomotion)。
启动与操作见 [C++ 仿真盲走说明](../../../doc/g1_blind_cpp.md)。
