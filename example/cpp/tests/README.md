# C++ 离线检查

正式构建不包含测试目标。使用 Python 标准库脚本临时生成测试项目，运行完成后删除测试
CMakeLists、可执行文件、对象文件、缓存和日志；失败或正常中断时也清理，不改变正式 build。

从仓库根目录执行：

```bash
# 离线检查：仿真 PD/挂带、手柄/相机/重置、实机保护及各策略
python3 example/cpp/tests/run_tests.py

# 只检查本次修改涉及的功能
python3 example/cpp/tests/run_tests.py --test gamepad --test locomotion
```

脚本默认只读 `example/cpp/build/CMakeCache.txt` 以复用本机 SDK2 和 ONNX Runtime 路径。
使用不同的正式构建目录可传 `--cache-dir /path/to/build`。
没有缓存的新机器可明确指定依赖：

```bash
python3 example/cpp/tests/run_tests.py \
  --sdk-prefix /opt/unitree_robotics \
  --onnxruntime-root /path/to/onnxruntime
```

手柄检查需要 `simulate/mujoco` 的头文件和动态库，但不需要实体手柄。
上述离线检查都不连接机器人，不启动 DDS，也不发送电机命令。
`g1_hardware.h`、`gamepad.h` 和推理头文件包含实际运行逻辑，不能作为“测试内容”删除。

`--test simulation` 检查当前关节状态 PD、躯干独立运动时的挂带阻尼、挂点重合/释放，
以及真实 G1 模型在连续 8 步补步中的悬挂初始化与保持稳定性。
Python 挂带行为检查仅需 numpy 和 mujoco，无需 SDK、pygame 或图形窗口：

```bash
python -B simulate_python/test/test_elastic_band.py
```

Python 检查同时覆盖 `g1_test` 与跑酷配置下未使能电机的 20 秒悬挂、被动动力学切换。
C++ 同一问题的回归入口为：

```bash
python3 example/cpp/tests/run_tests.py --test suspension-sim --verbose
```

它使用实际 C++ 观测类和挂带，分别检查 `g1_parkour`、`g1_test`、`g1_CA_PE_AMP_VAE`，
要求全部电机力矩为零且后 10 秒关节峰值速度低于 0.05 rad/s；同时验证启用控制时去掉额外
被动损耗、关闭控制后恢复。观测类初始化需要本机 UDP 套接字，但测试不发送观测或 DDS 电机命令。

同目录中的 MuJoCo 冒烟脚本用于完整链路验证，按对应控制器 README 的说明单独运行；
它们调用正式控制器，无需在正式 build 中保留任何测试可执行文件。

C++ 跑酷全链路检查（需要先编译正式 `g1_parkour`）：

```bash
python3 example/cpp/tests/run_tests.py --test parkour-sim
```

它在临时项目中编译 MuJoCo 测试驱动，调用实际控制器，依次检查固定基座复位/策略重入、
零速落地站立、停止观测后的清零保护、整机重置后保留策略状态、地形场景挂带下降/释放、地形前进，以及无挂带落地状态下的连续整机重置。使用本机回环 UDP 与 DDS domain 195..202，
不连接实机；运行结束后测试驱动、CMake 定义及日志全部自动清理。

逐帧数值核对（指定安装 numpy、mujoco、onnxruntime 的 Python）：

```bash
python3 example/cpp/tests/run_tests.py --test parkour-reference \
  --reference-python /path/to/python --verbose
```

40 帧逐项对比原生 UDP 状态/射线深度、本体与深度历史、双网络最终目标。
数值核对使用平地，以避免不同 MuJoCo 版本的高度场射线实现差异；地形闭环使用正式 C++ MuJoCo。
参考数据、测试可执行文件和构建目录在结束时自动清理。

只重跑指定闭环场景可使用 `--test parkour-sim --sim-mode ground-reset --verbose`。

调度抖动与实际 UDP 丢包检查：

```bash
python3 example/cpp/tests/run_tests.py --test parkour-sim --sim-mode jitter --verbose
```

在双脚落地、策略运行时暂停控制器进程 80ms 造成收包积压，再由 UDP 中继丢弃两批数据包；
要求无需重新启动策略即可继续站立，并保留原有高度/漂移判据。持续断流保护由 `stale` 场景检查。


CA PE AMP VAE 视觉策略检查：

```bash
python3 example/cpp/tests/run_tests.py --test ca-pe
python3 example/cpp/tests/run_tests.py --test ca-pe-sim
# 扩展至落地与丢帧恢复
python3 example/cpp/tests/run_tests.py --test ca-pe-sim --sim-mode ground --sim-mode jitter
```

`ca-pe` 默认包含在离线检查中。`ca-pe-sim` 运行正式 `build/g1_ca_pe_amp_vae`，
默认检查固定基座推理重入、落地站立、数据超时停机、整机重置、落地重置、丢帧恢复与前向行走；需先编译对应控制器。
两种视觉任务复用 C++ 仿真链路检查源码，由各自 JSON 指定物理步长、机体系和关节映射。

使用 `--verbose` 可查看每秒世界坐标 XY、航向角、机体系线速度和转向角速度，
以及 4.5～9 秒区间的平均速度。`walk` 的通过标准只覆盖前进和不倒地，不代表直线跟踪合格。
`body_vxy` 为状态刚体质心速度在完整 link 坐标系的表达；`heading_vxy` 为同一质心速度
在水平 yaw 坐标系的表达，与 CA PE 训练的线速度奖励一致。旧日志的 `body_vxy` 使用 link 原点速度。
`body_wz` 是局部 Z 角速度，`world_wz` 是世界 Z 分量，倾斜时不能混用。
同时输出 pelvis/torso 的世界系俯仰、waist pitch 实测角及平均目标；正俯仰表示前倾。
配置对照及后仰实测见[后仰排查](../../../doc/g1_ca_pe_backward_lean.md)。
对照不同部署 JSON 或 MJCF 时可使用 `--sim-config /path/to/config.json` 和
`--sim-scene /path/to/scene.xml`；模型目录仍使用正式控制器的默认目录。
`ca-pe` 另检查当前观测与 VAE 历史动作相差一拍的训练语义。

检查其他 CA PE 训练任务时，对离线和闭环检查均可传 `--task-dir example/exported/g1_test`，
同时替换配置和实际模型目录。历史动作时序按任务 JSON 的 `history_action_delay` 验证；
存在完整 `exported/policy.onnx` 时，离线检查另比较完整图与拆分双网络输出。

IMU 坐标和投影重力的数值检查（本机 UDP，不发送 DDS 电机目标）：

```bash
python3 example/cpp/tests/run_tests.py --test ca-pe-observation \
  --task-dir example/exported/g1_test --verbose
# 可选：直接使用另一框架的 SO3 模板计算投影重力
python3 example/cpp/tests/run_tests.py --test ca-pe-observation \
  --task-dir example/exported/g1_test --ctrlz-root /path/to/CtrlZ --verbose
```

覆盖 45 组混合姿态、独立腰部运动、实际 pelvis gyro、真实 UDP 观测与 actor 前 6 维输入，
可检出误用 MuJoCo 惯性主轴坐标 `mjOBJ_BODY` 或误选 torso 状态的问题。
同时检查 actor 的 `[vx,vy,wz]` 顺序和缩放在各姿态下不变，并用质心 Jacobian 核验日志使用的质心速度。
另用真实 UDP 相机输出核验 `camera.ray_alignment=body/yaw` 与解析平面深度，
确认 yaw 模式不会随 torso pitch 旋转射线，而原点仍随完整 torso 位姿移动。
完整坐标约定及闭环结果见[相机与速度坐标核查](../../../doc/g1_cpp_coordinate_audit.md)。

## C++ 网页监控检查

先编译正式工具 `cmake --build simulate/build --target joint_monitor -j4`，然后运行：

```bash
python3 example/cpp/tests/run_tests.py --test joint-monitor --verbose
```

覆盖数据有效期、目标优先级和回退、使能、IMU、采样限频与非法数据，
并启动正式 C++ 监控程序，通过实际 DDS 核验 HTTP JSON、两个并发 SSE 客户端、路由和退出。
需要本机 HTTP/DDS 套接字权限；不依赖 Python SDK。测试构建和日志在临时目录自动清理。
