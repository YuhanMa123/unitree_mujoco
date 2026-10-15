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
前向 `[-0.5, 1.0] m/s`、横向 `[-0.3, 0.3] m/s`、转向 `[-0.2, 0.2] rad/s`。
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

参数取自 [unitree_rl_lab 的 G1 velocity 部署配置](https://github.com/unitreerobotics/unitree_rl_lab/blob/main/deploy/robots/g1_29dof/config/policy/velocity/v0/params/deploy.yaml)，
本地参考版本 `4960b84`。PD 与裁剪已按用户提供的 WalkNet1 配置覆盖。
`g1_locomotion.yaml` 是可随项目一起使用的参数副本，
运行不依赖相邻的训练仓库。训练环境参见
[G1 29DOF locomotion](https://github.com/unitreerobotics/unitree_rl_lab/blob/main/source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/g1/29dof/velocity_env_cfg.py)。

当前 `policy.onnx` 已包含 `480 → 512 → 256 → 128 → 29` 的网络和 ELU 激活，
直接执行已导出的网络，不重新创建或训练模型。接口为 float32 `[1,480] → [1,29]`。

每帧 96 维观测，严格按以下顺序：

| 观测项 | 维度 | 处理 |
| --- | --- | --- |
| base_ang_vel | 3 | IMU 机体系角速度 × 0.2 |
| projected_gravity | 3 | 用 wxyz 四元数把世界重力方向 `[0,0,-1]` 旋转到机体系 |
| velocity_commands | 3 | 前向、横向、转向命令 |
| joint_pos_rel | 29 | 策略关节顺序的位置 − 训练默认位置 |
| joint_vel_rel | 29 | 策略关节顺序的速度 × 0.05 |
| last_action | 29 | 上一次网络输出的原始动作 |

各观测项分别保存 5 帧历史，由旧到新展开后再拼接，得到 480 维。
**不是**将完整的 96 维帧依次拼接。进入策略时历史用当前观测填充，上次动作清零；
退出后再次进入会重新初始化。

策略关节顺序通过 `joint_ids_map` 与 DDS 顺序转换。
目标为 `offset + scale × action`（默认 scale=0.25），每 20 ms 推理一次，
500 Hz 重复发布最近一次目标，由仿真器每个物理步更新 PD 力矩。
复位和策略阶段统一从 `g1_locomotion.yaml` 读取姿态与 PD，已移除独立 reset XML。
复位目标使用 `default_joint_pos`，按 `joint_ids_map` 转为 DDS 顺序，PD 直接使用
DDS 顺序的 `stiffness` 和 `damping`。默认增益与用户提供的 WalkNet1 一致：髋关节 Kp/Kd=100/2、
膝关节 150/4、踝关节 40/2、腰 yaw 200/5、腰 roll/pitch 40/5、肩/肘/腕 40/1。
WalkNet1 的 PD 数组按策略顺序解释，再通过 `joint_ids_map` 转换成 DDS 顺序。

WalkNet1 裁剪顺序为：观测先缩放再裁剪到 `[-100,100]`；网络原始动作裁剪到
`[-100,100]`，再乘动作缩放并加默认位置，最后将关节目标裁剪到 `[-5,5] rad`。
`last_action` 保留原始网络输出，在送入模型时同样经过观测裁剪。

控制周期保持 0.002 s，推理周期保持 0.02 s（50 Hz）。所贴配置中的
`PolicyFrequency: 10` 未提供调度实现，未将它直接解释为 10 Hz。

模型没有携带训练参数元数据，输入输出维度一致不能证明训练参数一致。
当前采用上游 G1 locomotion 观测结构和用户提供的 WalkNet1 PD/裁剪；若训练时修改了默认姿态、PD、
关节顺序或缩放，应同步提供/替换对应的部署 YAML。程序会校验模型形状、
关节映射、观测顺序、历史长度和数值有效性。

## 无界面 MuJoCo 集成检查

在已安装 `mujoco`、`unitree_sdk2py`、`pygame` 和 NumPy 的 Python 环境中执行。
脚本启动自己的仿真及控制器，默认使用隔离的本机 DDS domain `191`，不需要
先启动仿真窗口，也不要在该 domain 中运行其他控制器：

```bash
# 固定浮动基座，验证复位、使能、策略切换、速度输入、返回复位、WASD/QE 运动按键和空格退出
python example/cpp/tests/g1_locomotion_mujoco_smoke.py
# 进入策略后将机器人放到地面，验证站立、0.3 m/s 命令下的前移及停止
python example/cpp/tests/g1_locomotion_mujoco_smoke.py --ground
```

本机可使用 `/home/wsy/miniconda3/envs/unitree/bin/python`。
C++ 单元检查覆盖观测缩放、关节映射、历史排列、重入初始化、原始动作反馈、
四元数归一化、命令限幅、观测/动作/目标裁剪和非法观测。采用 WalkNet1 PD
后的地面检查中，0.3 m/s 命令持续约 4 秒，实际前移约 `0.066 m`，使能期间
基座最低高度约 `0.780 m`。机器人没有倒地，但实际位移明显小于命令对应位移；
这属于短时冒烟检查，
未证明完整速度范围内的跟踪精度或长期稳定性。
