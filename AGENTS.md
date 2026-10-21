# 项目约定

## 正式构建与测试清理

- 正式 `CMakeLists.txt` 只定义仿真器、控制器和实际使用的诊断工具；不要留下仅用于验证的 `add_executable`、`add_test`、CTest 或测试构建开关。
- 测试应当验证实际行为。保留有维护价值的测试源码和运行入口，不能通过删除保护逻辑、推理逻辑或测试覆盖来简化构建。
- C++ 离线检查统一使用 `python3 example/cpp/tests/run_tests.py`；可用 `--test gamepad`、`--test hardware`、`--test locomotion`、`--test parkour` 按需选择，可重复指定。
- 测试专用 CMakeLists、可执行文件、对象文件、缓存和 CTest 日志只允许放在独立临时目录中。完成测试后自动清理，失败或中断后也应清理；需要保留调试产物时先取得用户明确指示。
- 任务结束前检查正式 build 目录：不能遗留本次生成的测试可执行文件、测试目标目录或临时 CMake 定义。清理已确认仅供测试的产物属于用户长期授权，无需反复询问。
- 测试程序不是运行依赖；清理时保留仿真器、各任务推理程序、控制器内的关节初始化功能、实际诊断工具、运行库、配置和模型。不要按文件名模糊匹配或整目录删除用户数据。
- 一次性集成测试程序及临时测试脚本同样在验证结束后清理；保留必要的验证结论，并说明实际验证范围。

## 部署资源

- 推理部署参数与 ONNX 模型按任务统一放在 `example/exported/<task>/`，运行入口和文档使用该目录。
- 修改构建方式后重新配置并构建受影响的正式目标，完成与变更相关的检查；不能只删除测试文件而不验证正式程序仍可使用。

- G1 盲走仅使用 `example/cpp/g1_locomotion.cpp` 编译 `g1_locomotion`，不再增加独立关节复位可执行文件或双版本条件编译；关节初始化继续作为控制器内部流程保留。
- G1 跑酷主部署链路为 C++ 仿真器 `unitree_mujoco --parkour` 加 C++ 控制器 `g1_parkour`；配套配置为 `example/exported/g1_parkour/g1_torsobase_cfg.json`。Python 路径只作兼容与参考。完整 C++ 链路检查使用 `run_tests.py --test parkour-sim`，在临时目录构建并自动清理。
