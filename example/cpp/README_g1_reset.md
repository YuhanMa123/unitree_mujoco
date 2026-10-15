# G1 29DOF 关节角度复位

复位后接入项目 `policy.onnx` 进行 locomotion 推理，参见
[G1 locomotion 使用说明](README_g1_locomotion.md)。

`reset_g1_29dof.cpp` 通过 Unitree SDK2 的 `rt/lowstate` 读取当前关节角度，
以 500 Hz 发布 `unitree_hg::msg::dds_::LowCmd_` 到 `rt/lowcmd`。
启动后默认关闭命令下发。在控制程序所在终端先按 **9** 上电（使能命令输出），
再按 **P / p**（无需回车），生成 3 秒的 smoothstep 复位目标并向 `rt/lowcmd`
发布控制命令，目标到达后持续 PD 保持。未上电时复位按键会被拒绝。
未列出的关节目标为 **0 rad**。

| 关节 | 左侧目标（rad） | 右侧目标（rad） |
| --- | --- | --- |
| hip_pitch | -0.1 | -0.1 |
| knee | 0.3 | 0.3 |
| ankle_pitch | -0.2 | -0.2 |
| shoulder_pitch | 0.3 | 0.3 |
| shoulder_roll | 0.25 | -0.25 |
| elbow | 0.97 | 0.97 |
| wrist_roll | 0.15 | -0.15 |

这段代码用于关节复位，不包含浮动基座位置复位或站立平衡控制。
复位期间建议启用 MuJoCo 虚拟挂带，避免自由站立时跌倒。

## 编译

需要已安装 `unitree_sdk2`、Boost 头文件和 yaml-cpp，然后在仓库根目录运行：

```bash
cmake -S example/cpp -B example/cpp/build
cmake --build example/cpp/build --target reset_g1_29dof -j4
```

## 配置并运行

C++ 仿真器的 `simulate/config.yaml` 使用：

```yaml
robot: "g1"
robot_scene: "scene_29dof.xml"
domain_id: 1
interface: "lo"
enable_elastic_band: 1
```

如果使用 Python 仿真器，在 `simulate_python/config.py` 设置：

```python
ROBOT = "g1"
ROBOT_SCENE = "../unitree_robots/g1/scene_29dof.xml"
DOMAIN_ID = 1
INTERFACE = "lo"
ENABLE_ELASTIC_BAND = True
USE_JOYSTICK = 0
SIMULATE_DT = 0.002
```

Python 桥接在 DDS 回调中缓存命令，在每个物理步前使用当前 `qpos/qvel`
重新计算 PD 力矩，避免旧传感器状态及异步更新引起的手臂振荡。

## 网页实时监控

启动 G1 29DOF 仿真后，在安装了 `unitree_sdk2py` 的 Python 环境中另开终端，
从仓库根目录运行独立监控脚本：

```bash
python simulate_python/joint_monitor.py
# 可选：指定与仿真一致的 DDS 配置和网页端口
python simulate_python/joint_monitor.py --domain 1 --interface lo --port 8765
```

用浏览器打开 **http://127.0.0.1:8765**。
页面顶部和表格显示控制端 Enable（ON/OFF）；还显示关节实际/目标角度、误差、速度、反馈力矩和 Kp/Kd。
勾选关节行的复选框可同时查看多个关节最近 20 秒的曲线，颜色对应图例，实线为实际值、同色虚线为目标值，支持搜索、弧度/角度切换和暂停显示。
网页只查看数据，复位仍由 C++ 终端按键触发。

脚本独立订阅 `rt/lowstate`、`rt/lowcmd` 、目标话题 `rt/g1/reset_target` 和使能状态话题 `rt/g1/reset_enable`，不会向机器人发布控制命令，
不需要修改仿真脚本或导入 MuJoCo。默认最高 50 Hz 采样，通过 SSE 长连接推送，网页按屏幕刷新率绘制（通常 60 Hz）。
曲线保留收到的采样点，表格复用单元格，避免频繁重建。页面显示实测接收频率，
实际频率受 DDS 发布频率、系统负载和网络影响。可通过 `--rate 500` 将采样上限提高到 500 Hz。
力矩来自 `LowState.motor_state.tau_est`；目标和 PD 优先来自独立目标话题，未使能下发时也可显示；兼容从 `rt/lowcmd` 读取目标。
尚未收到目标，或目标超过 500 ms 未更新时，目标和误差显示为 —。
按 `p` 后 C++ 程序会向目标话题持续发布计算中的目标；该话题仅用于监控，
仿真器不订阅它；控制命令仅在完成上电并进入复位后发布。
页面时间为监控启动后的时间，数据中断会提示；监控脚本按 `Ctrl+C` 独立退出。
默认只允许本机访问，不需要安装额外 Web 库。脚本需要 G1 29DOF 消息及关节顺序；
Python 仿真发布的模式为 0 时无法自动辨别型号，请确认加载了正确场景。

启动所选仿真器后，在另一终端从仓库根目录运行：

```bash
./example/cpp/build/reset_g1_29dof
# 可选参数：复位秒数、网卡、DDS domain、XML 配置路径
./example/cpp/build/reset_g1_29dof 5 lo 1
./example/cpp/build/reset_g1_29dof 3 lo 1 example/cpp/g1_locomotion.yaml
```

按键在 **C++ 控制程序所在终端** 生效。
按键操作：

- `9`：上电，随后需按 `p` 开始复位。首次进入复位后，`9` 在本次进程中永久锁定。
- `p`：上电后从当前实测角度平滑复位。
- 空格 / Ctrl+C：清零控制增益及前馈力矩后退出。

复位中或保持中再次按 `p`，会重新读取当前关节角度并开始复位。
按 **空格** 或 `Ctrl+C` 退出，正在下发控制时清零增益和前馈力矩。
这里的“使能”是仿真控制程序的命令输出开关，不是实物电源控制。

必须加载 G1 的 `scene_29dof.xml`，不能使用 H2 场景。
本程序只接受正数 DDS domain，默认 `1`，不用于默认 domain `0` 的实物。
不要同时运行其他向 `rt/lowcmd` 发布的控制程序。
首次等待状态最多 5 秒；运行中状态超过 500 ms 未更新或机器人模式改变时，
程序清零控制增益和前馈力矩后退出。Ctrl+C 同样会释放关节控制。

## 修改默认姿态和 PD 参数

复位和推理统一读取 [`g1_locomotion.yaml`](g1_locomotion.yaml)，已移除独立 reset XML。
保存配置并重启控制程序即可，无需重新编译。第四个参数可指定其他 YAML 文件。

- `default_joint_pos`：29 个关节的复位目标（rad），采用策略顺序。
- `joint_ids_map`：策略索引到 DDS 关节索引的映射，复位读取时自动转换。
- `stiffness` / `damping`：29 个关节的 Kp / Kd，直接采用 DDS 顺序。

默认采用 WalkNet1 PD：髋关节 100/2、膝关节 150/4、踝关节 40/2、
腰 yaw 200/5、腰 roll/pitch 40/5、肩/肘/腕 40/1。
配置会检查长度、关节映射及数值有效性。

程序打印“插值完成”
表示目标轨迹到达默认姿态，不代表实际关节误差已严格为零。
Python 桥接的 `mode_machine` 默认为 0，程序无法据此辨别 23DOF/29DOF，
需要确保场景选择正确；C++ 桥接的非 29DOF 模式会被拒绝。

如果终端加载了 ROS 环境，且运行时在 DDS 初始化阶段出现 `free(): invalid pointer`，
检查是否混用了 ROS 与 Unitree 安装的 CycloneDDS 库。本机 Unitree 库位于
`/usr/local/lib`，可用以下方式运行：

```bash
LD_LIBRARY_PATH=/usr/local/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH} ./example/cpp/build/reset_g1_29dof
```

Enable 显示的是控制端输出开关，而非实物上电反馈。按 `9` 后显示 ON，退出后停止发布使能状态；
控制端退出或状态超过 500 ms 未更新后显示 —。未按 `p` 生成目标时也会发布使能状态。

网页 IMU 面板显示 `LowState.imu_state` 的四元数（w,x,y,z）、姿态 RPY、
三轴角速度和三轴加速度。RPY 从四元数计算，以兼容未填充 rpy 的 Python 桥接。
角度单位切换同时作用于 RPY 和角速度；加速度保持 m/s²（原始反馈，未去重力）。
缺失/无效的分量或数据超过 1 秒未更新时显示 —。
