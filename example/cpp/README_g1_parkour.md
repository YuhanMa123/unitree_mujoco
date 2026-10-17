# Python Unitree MuJoCo + C++ Parkour 推理

此入口直接连接当前仓库的 `simulate_python/unitree_mujoco.py`。
Python 运行 MuJoCo、Unitree SDK2 桥接和深度采集；C++ 运行两个 ONNX 网络及控制状态机。
不需要启动独立仿真 `example/parkour/run_mujoco.py`。

数据流：

```text
simulate_python/unitree_mujoco.py（现有 G1 场景、虚拟挂带、DDS 桥接）
    ├─ 同步关节/torso/深度观测 ──本机 UDP:17200──> g1_parkour（C++ ONNX Runtime CPU）
    └─ 每物理步 PD <──────────── rt/lowcmd DDS ────┘
```

## 1. 编译 C++

在仓库根目录运行：

```bash
cmake -S example/cpp -B example/cpp/build \
  -DBUILD_G1_PARKOUR=ON \
  -DONNXRUNTIME_ROOT=/home/wsy/rl/unitree_rl_lab/deploy/thirdparty/onnxruntime-linux-x64-1.22.0
cmake --build example/cpp/build --target g1_parkour g1_parkour_policy_test -j4
./example/cpp/build/g1_parkour --check-policy
```

`--check-policy` 仅离线推理，不初始化 DDS 或发送命令。
默认模型目录为用户指定的：
`/home/wsy/rl/InstinctLab/logs/instinct_rl/g1_parkour/20260905_091435/exported`。
默认配置 `example/parkour/g1.json`，支持 `--model-dir` 和 `--config` 覆盖。

## 2. 启动当前 Python 仿真

`simulate_python/config.py` 已启用 G1 29DOF 和 parkour 观测发送：

```python
ROBOT = "g1"
ROBOT_SCENE = "../unitree_robots/g1/scene_29dof.xml"
DOMAIN_ID = 1
INTERFACE = "lo"
ENABLE_ELASTIC_BAND = True
SIMULATE_DT = 0.002
PARKOUR_OBSERVATIONS = True
PARKOUR_OBSERVATION_PORT = 17200
PARKOUR_DEPTH_PREVIEW = True
```

在仓库根目录运行：

```bash
/home/wsy/miniconda3/envs/unitree/bin/python simulate_python/unitree_mujoco.py
```

Python 端不需要 ONNX Runtime、Torch 或 Isaac Sim。依赖原仿真所需的
MuJoCo、NumPy、pygame 和 unitree_sdk2py；深度用 CPU 射线采集。
默认另开OpenCV深度预览窗口：左侧为36×64原始图像平面深度，白框标出裁剪区域；
右侧为18×32裁剪、模糊并归一化后的最新编码器输入帧。暖色近、冷色远，原始图黑色代表无命中。
预览复用发送给C++的采集结果，无额外射线计算；右侧仅显示最新帧，完整编码器输入仍为8帧历史。
在预览窗口按Esc或关闭窗口只关闭预览，仿真和推理继续。
设置 `PARKOUR_DEPTH_PREVIEW = False` 可禁用；无桌面显示环境自动禁用。
预览需要带GUI支持的OpenCV（`cv2`），本机unitree环境已安装。
修改UDP端口时需同步 C++ `--port`；修改domain时同步 `--domain`。
切回其他控制程序可以将 `PARKOUR_OBSERVATIONS = False`，恢复原DDS控制路径。

## 3. 启动 C++ 推理控制器

关闭之前的复位/locomotion 控制程序，在第二个终端从仓库根目录运行：

```bash
./example/cpp/build/g1_parkour
# 可选参数
./example/cpp/build/g1_parkour --domain 1 --port 17200 --reset-seconds 3
```

控制终端操作：

1. 按 `9` 使能，再按 `p` 从当前角度平滑复位。
2. 等待 `Holding training pose`。在 MuJoCo 窗口按 `8` 调长虚拟挂带，使脚接近地面。
3. 回到控制终端按 `r`，看到 `Parkour policy ON` 后进入双网络推理。
4. MuJoCo 窗口的 `9` 切换挂带；控制终端的 `9` 仅用于使能。按键按焦点区分。

| 控制终端按键 | 功能 |
| --- | --- |
| `p` | 回到复位，清空策略历史 |
| `r` | 复位完成且各关节距默认角不超过0.35rad时进入策略 |
| `w` / `s` | 前向命令±0.1m/s，范围 `[0,1]` |
| `q` / `e` | 转向命令±0.1rad/s，范围 `[-1,1]` |
| `x` | 速度清零，策略继续运行 |
| 空格 / Ctrl+C | 清零输出并退出 |

横向命令固定为0，与训练配置一致。推理50Hz，DDS目标发布500Hz。
开启parkour观测时，Python额外按训练/模型较小力矩上限裁剪PD输出。
保留当前MuJoCo模型动力学参数，不自动替换惯量或关节阻尼。
训练机器人腰链为torso→pitch→roll→yaw→pelvis，MuJoCo为pelvis→yaw→roll→pitch→torso；
`g1.json` 的 `joint_sign` 对三个腰关节设为-1，其余为+1。
Python发送的q/dq先转到训练坐标，C++输出目标再转回MuJoCo/DDS坐标；原始网络动作反馈不反号。
控制器订阅 `rt/lowstate` 做存活与模式检查，策略输入使用同一个仿真时刻的
UDP完整观测，避免将pelvis IMU误作torso IMU。
现有网页监控继续使用 `simulate_python/joint_monitor.py`；控制器发布
`rt/g1/reset_target` 和 `rt/g1/reset_enable`。

## 验证与限制

```bash
ctest --test-dir example/cpp/build -R g1_parkour --output-on-failure
# 无界面检查同一 Python Unitree 桥接、C++网络和DDS控制；基座固定
/home/wsy/miniconda3/envs/unitree/bin/python example/cpp/tests/g1_parkour_mujoco_smoke.py
# 释放浮动基座，验证零命令落地站立
/home/wsy/miniconda3/envs/unitree/bin/python example/cpp/tests/g1_parkour_mujoco_smoke.py --ground
# 停止观测，检查C++超时后清零输出
/home/wsy/miniconda3/envs/unitree/bin/python example/cpp/tests/g1_parkour_mujoco_smoke.py --stale
```

测试使用本机DDS domain192和临时UDP端口，不要在该domain运行其他控制器。
已通过C++历史排列、深度抽帧、预处理、动作反馈与协议检查；
固定基座完整链路支持复位、策略重入和停止。默认观测下，C++与独立Python参考推理
目标角最大差异约 `5.1e-7 rad`。

控制中UDP观测超过150ms未更新、DDS状态超过500ms未更新、仿真重置、观测丢帧
导致历史时序不连续、推理超过20ms或策略中torso高度低于0.5m/倾角超过1rad，
均停止并清零。暂停仿真会触发停止；恢复后需重启控制器。
UDP仅绑定/发送到127.0.0.1，协议为小端；Python与C++使用同一份配置中的关节顺序，
包头校验关节名称及方向签名，不匹配则拒绝。

零命令落地检查在当前Python Unitree→C++→DDS链路上运行19.238秒仿真时间，
最低torso高度0.775m、最大XY漂移0.283m，未倾倒。可重复执行延长检查：

```bash
/home/wsy/miniconda3/envs/unitree/bin/python example/cpp/tests/g1_parkour_mujoco_smoke.py --ground --stand-seconds 20
```

完整跑酷和速度跟踪仍未验证。修改关节方向配置后须同时重启Python仿真和C++控制器。
训练模型以torso为根且带鞋，当前MuJoCo机器人几何、惯量、关节被动阻尼
和相机遮挡存在差异；导出目录缺少训练参数快照，部署参数由本机训练源码重建。
模型接口、参数来源及差异见 [网络与配置说明](../parkour/README.md)。

## 文件与网络接口

| 文件 | 用途 |
| --- | --- |
| `example/cpp/g1_parkour.cpp` | C++控制状态机、UDP接收、DDS命令发布 |
| `example/cpp/g1_parkour_policy.h` | 双网络推理、本体历史、深度预处理及动作映射 |
| `example/parkour/g1.json` | 关节顺序、方向、默认角、PD和动作缩放 |
| `simulate_python/parkour_observation.py` | 在当前仿真采集同步状态与深度，发送UDP观测 |
| `example/parkour/mujoco_adapter.py` | MuJoCo关节访问、torso状态和相机射线 |
| `simulate_python/depth_preview.py` | 原始深度及最新编码器帧预览 |
| `example/parkour/policy.py` | 可选的Python参考推理，主部署流程不调用 |

| 网络 | 输入 | 输出 |
| --- | --- | --- |
| `0-depth_encoder.onnx` | float32 `[1,8,18,32]` | float32 `[1,128]` |
| `actor.onnx` | float32 `[1,896]` | float32 `[1,29]` |

动作网络的896维输入由768维本体历史和128维深度特征组成。
本体观测按角速度、重力、速度命令、相对关节角、关节速度、上一原始动作顺序拼接；
各项单独保留8帧，由旧到新展开。
深度来自当前场景group 0碰撞几何的36×64射线，单位米，使用图像平面深度。
裁掉上方18行和左右各16列，再做3×3、sigma=1的高斯模糊，
裁剪到0～2.5m并归一化。缓存37帧，按距当前35、30、25、20、15、10、5、0帧抽取8张图。
初次进入策略使用当前观测填充历史，再次进入时重新初始化。

## 常见问题

| 现象 | 检查项 |
| --- | --- |
| 一直等待观测 | Python是否已启动、是否启用`PARKOUR_OBSERVATIONS`，两端UDP端口及DDS domain是否一致 |
| UDP端口绑定失败 | 是否还有另一个`g1_parkour`进程占用端口 |
| 按`r`没有进入策略 | 先按`9`和`p`，等待复位完成；保持挂带以使关节接近训练默认姿态 |
| 更新后关节方向签名不匹配 | 两端使用同一份`g1.json`，重新编译C++并同时重启仿真与控制器 |
| 没有深度窗口 | 检查`PARKOUR_DEPTH_PREVIEW`、桌面显示环境以及带GUI的OpenCV；关闭后需重启仿真重新打开 |
| 画面呈大块色块 | 原始分辨率仅36×64，预览使用最近邻放大；右侧是18×32最新帧 |
| 暂停仿真后控制器退出 | 观测超时保护的预期行为，恢复仿真后重启控制器 |

模型文件使用外部导出目录，不随本次代码提交复制；换机器时通过`--model-dir`指定路径。
原有`g1_locomotion`使用根目录`policy.onnx`，与本入口的双网络模型分别运行。
