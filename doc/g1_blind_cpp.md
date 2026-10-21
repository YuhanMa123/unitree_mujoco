# G1 29DOF 盲走：C++ MuJoCo + C++ DreamWaQ

当前使用 `example/exported/g1_locomotion/policy.onnx` 和 `example/exported/g1_locomotion/g1_locomotion.yaml`。
策略只读取关节位置、关节速度、IMU、速度命令及历史观测，不需要深度相机。
`example/exported/g1_parkour/` 中的深度编码器和动作网络用于另一套跑酷策略。
Xbox 手柄操作及参数见 [G1 Xbox 控制说明](g1_xbox_cpp.md)。

## 当前配置

`simulate/config.yaml` 已设置为 G1 29DOF 平地场景：

```yaml
robot: "g1"
robot_scene: "scene_29dof_flat.xml"
domain_id: 1
interface: "lo"
idl_type: 1
use_joystick: 0
enable_elastic_band: 1
```

平地场景引用原有 `g1_29dof.xml`，关节、电机和传感器定义保持相同。
初始开启挂带用于关节复位。C++ 仿真器已补上关闭挂带时清除残留外力的逻辑。

## 启动

先退出之前的 `stand_go2`、复位程序或其他控制器，避免同时向 `rt/lowcmd` 下发命令。

终端 A，从仓库根目录启动仿真器：

```bash
cd /home/mayh/MYH/Project_code/Python/unitree_mujoco
./simulate/build/unitree_mujoco
```

不要继续使用以前命令中的 `-r go2` 或 `-s scene_terrain.xml`。
等待 G1 场景加载完毕，再在终端 B 运行：

```bash
cd /home/mayh/MYH/Project_code/Python/unitree_mujoco
./example/cpp/build/g1_locomotion 3 lo 1 example/exported/g1_locomotion/g1_locomotion.yaml example/exported/g1_locomotion/policy.onnx
```

位置参数依次为：复位时间（秒）、网卡、DDS domain、部署配置、ONNX 模型。
不带参数运行 `./example/cpp/build/g1_locomotion` 也使用相同默认值。

## 进入策略

1. 在终端 B 按 `9`，使能控制命令输出。
2. 在终端 B 按 `p`，等待约 3 秒，直到出现 `Holding default pose`。
3. 点击 MuJoCo 窗口，按 `8` 逐步放长挂带，让双脚接触地面。
4. 回到终端 B，按 `r`，出现 `Locomotion policy ON`，初始速度为零。
5. 点击 MuJoCo 窗口，按 `9` 关闭挂带，观察机器人独立站立。
6. 回到终端 B，按 `w` 一次尝试 0.1 m/s 前进；稳定后再逐步增加。

先让双脚接地再进入策略，避免策略长时间在空中运行。
所有控制按键直接生效，无需回车；MuJoCo 窗口和终端 B 的按键功能不同。

## 按键

终端 B：

| 按键 | 功能 |
| --- | --- |
| `9` | 使能命令输出；开始复位后此键锁定 |
| `p` | 从当前关节角度复位；策略运行时按此键会退出策略并复位 |
| `r` | 复位完成后进入策略，速度命令清零 |
| `w` / `s` | 前向速度增加 / 减少 0.1 m/s |
| `a` / `d` | 横向速度增加 / 减少 0.1 m/s；正方向向左 |
| `q` / `e` | 转向角速度增加 / 减少 0.1 rad/s；正方向左转 |
| `x` | 所有速度命令归零，策略继续运行 |
| 空格 / Ctrl+C | 清零控制增益和前馈力矩，退出控制程序 |

速度命令累计并保持。例如从零连续按三次 `w` 得到 0.3 m/s，按一次 `s` 后为 0.2 m/s。
前向限幅 `[-0.6, 1.0] m/s`，横向 `[-0.5, 0.5] m/s`，转向 `[-1.57, 1.57] rad/s`。
日常停止行走用 `x`；空格或 Ctrl+C 会撤去关节控制，未被挂带支撑的机器人可能倒下。

MuJoCo 窗口：

| 按键 | 功能 |
| --- | --- |
| `8` | 放长挂带，降低机器人 |
| `7` | 缩短挂带，抬高机器人 |
| `9` | 切换挂带开关 |

重新开始完整测试时，退出控制器和仿真器，再按上述顺序启动。

## 重新编译

当前可执行文件已经构建。需要重建时，在仓库根目录执行：

```bash
ORT_ROOT=/home/mayh/MYH/Project_code/Python/unitree_rl_lab/deploy/thirdparty/onnxruntime-linux-x64-1.22.0
cmake -S example/cpp -B example/cpp/build \
  -DCMAKE_PREFIX_PATH="$PWD/.local/unitree_robotics/lib/cmake" \
  -DBUILD_G1_LOCOMOTION=ON \
  -DONNXRUNTIME_ROOT="$ORT_ROOT" \
  -DONNXRUNTIME_INCLUDE_DIR="$ORT_ROOT/include" \
  -DONNXRUNTIME_LIBRARY="$ORT_ROOT/lib/libonnxruntime.so.1.22.0"
cmake --build example/cpp/build --target g1_locomotion -j4
cmake --build simulate/build --target unitree_mujoco -j4
```

显式指定 ONNX Runtime 库可避免系统的 1.17 库与 1.22 头文件混用。
运行时请保留该 ONNX Runtime 目录和仓库内 `.local/unitree_robotics`。

离线策略检查不启动 DDS、不下发命令：

```bash
./example/cpp/build/g1_locomotion --check-policy
python3 example/cpp/tests/run_tests.py --test locomotion
```

## 本次检查范围

仿真器和控制器均已成功编译，离线推理及现有 C++ 策略检查通过。
使用临时无界面检查程序加载同一个平地模型和仓库的 C++ `G1Bridge`，
在隔离的 loopback DDS domain 193 中完成了复位、策略启动、0.3 m/s 前进、
速度清零和退出检查。前进约 4 秒、位移约 1.12 m；控制期间基座最低约 0.77 m，未跌倒。
检查中通过固定/释放浮动基座模拟启动过程，未自动操作图形窗口或挂带按键。
这是短时检查，没有覆盖长期稳定性、全部速度范围或复杂地形。
