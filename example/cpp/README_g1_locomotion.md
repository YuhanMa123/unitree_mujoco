# G1 29DOF：复位后运行 ONNX locomotion

机载实机部署入口、只读核验和保护设置见 [实机部署说明](README_g1_hardware.md)。

`g1_locomotion` 与 `reset_g1_29dof` 共用复位、使能和 DDS 发布逻辑，增加
ONNX Runtime CPU 推理。默认加载仓库根目录的 `policy.onnx`；不需要 Isaac Sim。
原来的复位程序仍可单独编译使用。

## 编译

依赖 Unitree SDK2、Boost、yaml-cpp 和 ONNX Runtime C/C++ 发行包。
在本机仓库根目录执行：

```bash
cmake -S example/cpp -B example/cpp/build \
  -DBUILD_G1_LOCOMOTION=ON \
  -DONNXRUNTIME_ROOT=/home/wsy/rl/unitree_rl_lab/deploy/thirdparty/onnxruntime-linux-x64-1.22.0
cmake --build example/cpp/build --target g1_locomotion g1_locomotion_policy_test -j4
```

其他机器将 `ONNXRUNTIME_ROOT` 改为包含 `include/onnxruntime_cxx_api.h` 和
`lib/libonnxruntime.so`（或 `libonnxruntime.so.1`）的发行包目录。
运行时需要保留该发行包。Linux 构建优先使用链接时选定的 SDK/ORT 动态库，
避免已经加载 ROS 环境的终端混用不同版本的 DDS 或 ONNX Runtime。

先做离线推理检查（不初始化 DDS，不下发命令）：

```bash
./example/cpp/build/g1_locomotion --check-policy
ctest --test-dir example/cpp/build --output-on-failure
```

## 运行

1. 按 [复位说明](README_g1_reset.md) 启动 G1 `scene_29dof.xml` 仿真，
   DDS domain 为 `1`、网卡为 `lo`，初始启用虚拟挂带。
2. 关闭之前的复位/控制程序，在另一终端运行：

   ```bash
   ./example/cpp/build/g1_locomotion
   ```

3. 在控制终端按 `9` 上电，再按 `p` 复位，等待 `Holding default pose`。
4. 在 MuJoCo 窗口按 `8` 调长挂带，使双脚接触地面；在控制终端按 `r` 进入策略。
   策略以零速度命令开始运行；在 MuJoCo 窗口按 `9` 关闭挂带后观察站立，再输入行走速度。

控制终端的按键：

| 按键 | 行为 |
| --- | --- |
| `p` | 上电后从当前实测角度平滑复位；策略运行时退出策略并重新复位 |
| `9` | 上电（使能命令输出）；仅进入复位之前有效 |
| `r` | 必须先按 `p` 且复位完成，才能启动策略；29 个关节需距离训练默认姿态各不超过 0.35 rad |
| `w` / `s` | 前向速度增加/减少 0.1 m/s |
| `a` / `d` | 横向速度增加/减少 0.1 m/s |
| `q` / `e` | 转向速度增加/减少 0.1 rad/s |
| `x` | 将三维速度命令清零，继续运行策略 |
| 空格 / Ctrl+C | 清零并退出 |

速度按键仅在策略运行时生效，命令会保持到下次按键。命令限幅来自 YAML：
前向 `[-0.6, 1.0] m/s`、横向 `[-0.5, 0.5] m/s`、转向 `[-1.57, 1.57] rad/s`。
严格按 `9 → p → r` 操作，复位期间提前按 `r` 会被拒绝。首次进入复位后，
`9` 在本次进程中永久锁定，不能通过重复上电打断复位或推理。
推理中可按 `p` 返回复位，完成后再按 `r`。`d` 和 `q` 均为运动按键，不退出或关闭输出。
控制终端的 `9` 表示命令输出使能；MuJoCo 窗口的 `9` 仍用于切换挂带，二者按焦点区分。

可选位置参数：

```bash
./example/cpp/build/g1_locomotion 3 lo 1 \
  example/cpp/g1_locomotion.yaml policy.onnx
#                         复位秒数 / 网卡 / domain / 统一配置 / 模型
```

保持原有正数 domain 限制、500 ms 状态超时和模式检查；推理异常或无效观测也会
清零并退出。该控制器用于 MuJoCo 仿真。复位保持不包含浮动基座平衡控制。
启动策略前不要同时运行其他向 `rt/lowcmd` 发布命令的控制程序。

网页监控仍使用：

```bash
python simulate_python/joint_monitor.py
```

目标话题继续为 `rt/g1/reset_target`，使能状态为 `rt/g1/reset_enable`，
因此复位和策略目标都能在现有网页查看。

## 模型接口

当前模型来自：

```text
/home/wsy/rl/lab_isaac3/logs/rsl_rl/g1_dwaq/2026-10-15_23-09-27/exported/policy.onnx
SHA256: 4f23975849ef66d73a170ee3435a274a4c42dd31830f8b07161add84846a871d
```

该文件已复制为仓库根目录的 `policy.onnx`，运行时不依赖训练仓库。
`g1_locomotion.yaml` 按同一训练目录的 `params/env.yaml` 和 `params/agent.yaml`
同步默认姿态、PD、观测缩放、动作裁剪及命令范围。

这是包含观测归一化和上下文编码器的 DreamWaQ 模型，直接执行导出的网络：

| 张量 | 类型与形状 | 含义 |
| --- | --- | --- |
| observation | float32 `[1,96]` | 当前观测 |
| observation_history | float32 `[1,480]` | 含当前帧的五帧历史 |
| actions | float32 `[1,29]` | 未缩放的关节动作 |

控制器也支持原来的单输入 `[1,480] → [1,29]` 模型，但更换模型时须配套部署参数。

每帧 96 维观测，严格按以下顺序：

| 观测项 | 维度 | 处理 |
| --- | --- | --- |
| base_ang_vel | 3 | IMU 机体系角速度，scale=1 |
| projected_gravity | 3 | 用 wxyz 四元数把世界重力方向 `[0,0,-1]` 旋转到机体系 |
| velocity_commands | 3 | 前向、横向、转向命令 |
| joint_pos_rel | 29 | 策略关节顺序的位置 − 训练默认位置 |
| joint_vel_rel | 29 | 策略关节顺序的速度 × 0.05 |
| last_action | 29 | 上次动作裁剪到 `[-100,100]` 后的值，未乘 0.25 |

观测缩放后裁剪到 `[-100,100]`。各观测项分别保存 5 帧历史，
由旧到新展开后再拼接，得到 480 维；当前 96 维输入单独按上表拼接。
进入策略时历史用当前观测填充，上次动作清零；退出后再次进入会重新初始化。
部署不添加训练时的观测噪声，归一化由 ONNX 网络内部执行。

策略关节顺序通过 `joint_ids_map` 与 DDS 顺序转换。
目标为 `default_joint_pos + 0.25 × clip(action, -100, 100)`，
与此次训练一致，不再使用旧配置的 `[-5,5] rad` 目标裁剪。
复位和策略阶段统一从 YAML 读取姿态与 PD；PD 数组使用 DDS 顺序：
髋 pitch/yaw Kp/Kd=50/2.5、髋 roll 75/2.5、膝 100/4、踝 40/2、
腰 yaw 100/5、腰 roll/pitch 75/5、肩/肘/腕 40/3。

控制周期 0.002 s，推理周期 0.02 s（训练 dt=0.005、decimation=4）。
500 Hz 重复发布最近一次目标，由仿真器每个物理步更新 PD 力矩。

## 无界面 MuJoCo 集成检查

在已安装 `mujoco`、`unitree_sdk2py`、`pygame` 和 NumPy 的 Python 环境中执行。
脚本启动自己的仿真及控制器，默认使用隔离的本机 DDS domain `191`，
不要在该 domain 中运行其他控制器。无需 Isaac Sim。

```bash
# 固定浮动基座，检查复位、策略切换、速度按键和退出
/home/wsy/miniconda3/envs/unitree/bin/python example/cpp/tests/g1_locomotion_mujoco_smoke.py
# 落地后检查短时站立、0.3 m/s 前进和停止
/home/wsy/miniconda3/envs/unitree/bin/python example/cpp/tests/g1_locomotion_mujoco_smoke.py --ground
```

C++ 检查覆盖观测缩放、关节映射、历史排列、重入初始化、动作反馈、
四元数归一化、命令限幅、观测/动作裁剪和非法观测。
离线默认姿态推理已与独立 Python ONNX Runtime 结果对齐。

此次 DreamWaQ 模型通过固定基座及落地检查：0.3 m/s 命令持续约 4 秒，
实际前移约 `1.04 m`，检查期间基座最低高度约 `0.770 m`，未倒地。
这是短时冒烟检查，未验证长期稳定性或完整速度范围的跟踪精度。
