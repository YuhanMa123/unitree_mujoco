# C++ 视觉仿真的相机和速度坐标核查

核查对象：`g1_CA_PE_AMP_VAE`、`g1_test` 两个任务，以及共享的 C++ 视觉仿真/控制器代码。未修改正式任务参数、模型、PD 或控制输入。

## 相机

两个任务均指定 `state_body=pelvis`、`camera.body=torso_link`，均未指定 `camera.ray_alignment`，所以使用默认 `body`。相机位置和射线方向为：

```
p_world = p_torso + R_torso * p_offset
d_world = R_torso * R_mount * d_camera
```

因此相机跟随 torso 的完整 roll/pitch/yaw，也跟随腰部关节引起的 torso 姿态变化。pelvis 状态观测与 torso 相机挂载互相独立。显式设置 `ray_alignment=yaw` 才会让射线只随航向旋转；该诊断选项的相机原点仍随完整 torso 位姿移动。

## 指令和观测

指令按 `[vx, vy, wz]` 进入 actor 当前观测第 6～8 项（零起始索引），除配置限幅外没有旋转、换轴或额外缩放。

| 量 | 与训练一致的含义 |
| --- | --- |
| vx/vy 指令 | m/s，pelvis 航向对应的水平坐标系，X 向前、Y 向左；不随 pelvis roll/pitch 倾斜 |
| wz 指令 | rad/s，与 pelvis 局部 Z 角速度分量比较；直立时正值表示左转 |
| 角速度观测 | pelvis link 坐标系的三轴角速度；actor 输入乘配置中的 0.2 |
| 重力观测 | `R_pelvis^T * (0,0,-1)` |
| 相机姿态 | torso 完整姿态与固定安装外参的组合 |

训练依据是两个任务保存配置中绑定的 RobotLab `track_lin_vel_xy_yaw_frame_exp` 和 `track_ang_vel_z_exp`：前者将根刚体世界系质心线速度转到 yaw 坐标系，后者直接比较 `root_ang_vel_b[:,2]`。所以前两项与第三项的参考系定义确实不同，这是训练本身的约定。

机器人航向为 90° 时，正 vx 对应世界 +Y。不能再次用 torso 或世界航向旋转指令。零 wz 表示零局部 Z 转速目标，并不是保持某个绝对航向；倾斜且存在其他轴转动时，局部 wz 也不等于欧拉 yaw 的时间导数。

MuJoCo 角速度使用 `mjOBJ_XBODY` 加局部输出，正确对应 link 轴。`mjOBJ_BODY` 的局部输出使用惯性主轴，不能直接替代。CA PE actor 没有输入实测基座线速度；仿真器经策略和关节 PD 形成运动，不直接设置基座速度。

老 `g1_parkour` 默认使用 torso 状态，角速度缩放为 0.25，指令范围也不同；这些是另一任务的约定，不能混用 CA PE 参数。

## 诊断日志口径修正

原闭环测试的 `body_vxy` 取 link 原点速度，并在完整倾斜机体系表达。它与训练线速度奖励使用的质心速度、yaw 坐标系不同，不能直接用于精确判断奖励跟踪误差。

本次仅修改测试诊断：先用 `mjOBJ_BODY` 世界输出取得质心速度，再分别用 link 姿态和 yaw 旋转得到 `body_vxy`、`heading_vxy`；同时输出局部 `body_wz` 和 `world_wz`。旧报告中的 `body_vxy` 属于旧口径。

## 验证与限制

通过入口：

```
python3 example/cpp/tests/run_tests.py --test ca-pe-observation --test ca-pe-sim \
  --task-dir example/exported/g1_test --sim-mode walk \
  --ctrlz-root /home/mayh/MYH/Project_code/C++/CtrlZ --verbose
```

45 组混合姿态及独立腰部运动覆盖真实 UDP、IMU、actor 指令和相机平面深度；最大角速度误差 `7.18e-9`，重力误差 `1.79e-7`，相机平面误差 `5.36e-8`。质心速度与质心 Jacobian 结果最大误差 `2.78e-17`，该组姿态中 link 原点与质心速度差最高约 `0.0157 m/s`。另外通过 CA PE ONNX 离线一致性与游戏手柄检查。

`g1_test` 平地闭环，指令 `(0.5,0,0)`，4.5～9 秒平均：

| 测量 | 结果 |
| --- | --- |
| 航向系质心 vx/vy | `0.501572 / -0.00804183 m/s` |
| pelvis 局部 wz | `-0.1881 rad/s` |
| 世界系 wz | `-0.18821 rad/s` |
| pelvis/torso 俯仰 | `-7.0929° / -7.10947°`（正值为前倾） |

当前坐标和指令链路检查通过，但实际存在右转漂移和后仰。`walk` 通过标准仅覆盖前进和不倒地，不能据此宣布直线或转向跟踪合格。本次结果只覆盖该模型、场景和前进指令，未证明所有速度方向和复杂地形的跟踪性能。

所有测试在临时构建目录执行，退出时自动清理；未向正式 CMake 添加测试目标。
