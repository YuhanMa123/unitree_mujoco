# G1 盲走：Xbox 手柄控制

使用 C++ 仿真器和 C++ `g1_locomotion`，不需要 Python SDK。
本机手柄为 Microsoft Xbox Series S|X Controller，设备 `/dev/input/js0`。
程序通过 Linux joystick ioctl 获取轴/按键映射，两个进程分别读取同一手柄。
手柄操作不依赖窗口或终端的键盘焦点，原有键盘控制仍可用。

## 启动与操作顺序

退出旧控制器和旧仿真器，在终端 A 启动：

```bash
cd /home/mayh/MYH/Project_code/Python/unitree_mujoco
./simulate/build/unitree_mujoco
```

等待 G1 场景加载完成，在终端 B 启动：

```bash
cd /home/mayh/MYH/Project_code/Python/unitree_mujoco
./example/cpp/build/g1_locomotion
```

1. 松开各按键，让摇杆回中。两个终端都应显示 `gamepad connected`。
2. 按 **B**，自动使能命令输出并复位关节。等待约 3 秒，直到显示 `Holding default pose`。
3. 按住 **A** 逐步降低挂带，使双脚接地；**Y** 用于抬高。
4. 按 **Menu/Start（≡）** 启动盲走策略，初始速度为零。
5. 按 **X** 释放挂带，观察独立站立，再拨动摇杆或使用十字键。
6. 停止行走时按 **View/Back（两个方框）** 或 **右摇杆按下 R3**。

B 复位的是关节姿态，并在当前位置重新挂起，保留当前世界位置。进入策略需等待复位完成，
不能跳过 B 直接用 Menu 启动。首次连接、复位、启动、停止和重连后，
需要摇杆回中后才接受新的摇杆运动。

## 按键映射

| 输入 | 功能 |
| --- | --- |
| 左摇杆前后 | 机器人 base 坐标系前后线速度，回中后归零 |
| 左摇杆左右 | base 坐标系左右线速度，回中后归零 |
| 右摇杆左右 | 转向角速度；左推左转，回中后归零 |
| 十字键上 / 下 | 当前前向速度增加 / 减少 0.1 m/s |
| 十字键左 / 右 | 当前左向速度增加 / 减少 0.1 m/s |
| B | 重新挂起、自动使能并平滑复位；行走中按 B 会退出策略并复位 |
| Y（按住） | 挂带抬高机器人 |
| A（按住） | 挂带降低机器人 |
| X | 释放挂带 |
| LB（左上肩键） | 切换居中跟随；开启时切到背后，之后持续居中，鼠标自由旋转/缩放 |
| LT（左下扳机） | 压过一半并持续 0.35 秒：从机器人后方跟随其朝向；松开停止朝向跟随，居中由 LB 决定 |
| Menu/Start | 复位完成后启动策略 |
| View/Back | 所有速度归零，策略继续运行 |
| R3（右摇杆按下） | 同 View/Back，停止行走 |
| RB（右上肩键） | 整机仿真重置（同 Backspace）：恢复初始位置、姿态、速度和仿真时间；保持策略运行状态 |

LB 按一次开启居中跟随，并切到当前机器人背后；之后只更新相机中心位置，
机器人转向不会强制旋转相机，鼠标仍可调整角度与缩放。再按一次 LB 关闭居中跟随，
按住不会重复切换。手柄断开时关闭居中跟随，重连后可按 LB 重新开启。
LT 压过一半并持续 0.35 秒后跟随机器人朝向；松开保留最后角度，鼠标恢复自由调整。
如果 LB 已开启，松开 LT 后仍保持机器人居中；LB 关闭时则停止位置跟随。

使用 Y/A 时，如果正在行走，会先清零速度并切回关节复位/姿态保持。
再次行走时，等待复位完成、双脚接地，再按 Menu 和 X。
不运行控制器时，挂带、相机和 RB 仿真重置仍可由仿真器单独处理。

RB 每次按下只重置一次，按住不会重复重置，也不再用于重新挂起。
策略未启动时按 RB，不会自动使能输出或启动策略；正在关节复位/保持时会从重置后的角度重新插值。
策略运行中按 RB，策略保持运行，清空旧观测历史、上一动作和速度命令，使用重置后的状态继续推理。
挂带开关及长度保持当前设置，挂点水平位置回到初始机器人上方；RB 不替代 B 的关节复位功能。
上述状态保留适用于 C++ 仿真器配合 `g1_locomotion` 或 `g1_parkour`。跑酷任务同样支持 B 复位与 Menu 启动策略；其横移和倒退固定为 0，RB 重置仍由仿真器处理。

十字键是按压边沿触发，按住不会连续加速，松开后速度保持。例如连续点三次“上”，
得到 0.3 m/s。拨动左摇杆会接管线速度并清除十字键巡航，摇杆回中后停止。
若左摇杆仍偏转时按十字键，先切换为十字键巡航；左摇杆回中再拨动后才能重新接管。
右摇杆独立控制转向，不改变十字键保持的线速度。

View/R3 清零速度并取消巡航。手柄拔出时也会清零速度，保持零速度策略；
程序每秒尝试重连，重连时不会执行仍被按住的按钮，也不会恢复旧巡航速度。
控制器的手柄输入仅在 MuJoCo 模式启用，`--hardware` 模式沿用原有键盘流程。

## 参数

共享配置位于 `simulate/gamepad.yaml`，修改后重启两个程序即可：

| 参数 | 默认值 |
| --- | --- |
| enabled | true |
| device | /dev/input/js0 |
| deadzone | 0.12，摇杆死区 |
| forward_max / backward_max | 0.6 / 0.4 m/s |
| lateral_max | 0.4 m/s |
| yaw_max | 1.0 rad/s |
| velocity_step | 0.1 m/s |
| band_speed | 0.4 m/s |
| heading_hold_seconds | 0.35 s |
| camera_distance / camera_elevation | 2.5 m / -15° |

`simulate/config.yaml` 的 `use_joystick` 是原有 Unitree 无线手柄话题发布选项；
这次的直接 Xbox 输入由 `gamepad.yaml` 控制，不需要开启原话题发布。

## 只读检查及重新编译

只检查手柄，不启动 DDS、不发机器人命令：

```bash
./example/cpp/build/gamepad_monitor --once
# 持续显示摇杆及按键变化，Ctrl+C 退出：
./example/cpp/build/gamepad_monitor
```

从仓库根目录重新编译与检查：

```bash
cmake -S example/cpp -B example/cpp/build
cmake --build example/cpp/build --target g1_locomotion gamepad_monitor -j4
cmake --build simulate/build --target unitree_mujoco -j4
python3 example/cpp/tests/run_tests.py --test gamepad --test locomotion
```

上述命令使用当前机器已经配置好的 SDK2/ONNX Runtime CMake 缓存。
全新构建目录的 SDK2 和 ONNX Runtime 参数见 [C++ 盲走说明](g1_blind_cpp.md)。
空格或 Ctrl+C 仍会退出控制器并撤去关节控制；正常停止行走请使用 View/R3 或键盘 x。

## 检查范围

已成功编译仿真器、盲走控制器、只读手柄监视器及自动检查程序。
`g1_gamepad` 检查覆盖 Linux 输入码映射、单次按压、摇杆/巡航接管、限幅、
重连与回中要求、挂带、相机实际观察方向；`g1_locomotion_policy` 和离线推理检查通过。
只读监视器成功读取本机 Xbox 的真实设备，静止漂移处于配置的死区内。
C++ DDS/同一 MuJoCo 模型的无界面行走回归通过：0.3 m/s 命令约 4 秒，前移约 1.11 m，
控制期间基座最低约 0.77 m，未跌倒。该回归通过脚本发送键盘指令，手柄动作逻辑由自动检查覆盖；
图形窗口中的完整手动手柄操作尚需实际试用。

RB 重置回归已通过无界面 C++ MuJoCo/G1Bridge 与实际 `g1_locomotion` 联调：未使能和关节保持阶段重置后策略均保持 OFF，策略运行中连续两次重置均保持 ON，之后仍能接收速度命令。该检查固定浮动基座，验证状态与推理链路，未验证重置后的自由落地稳定性或实体手柄操作。
