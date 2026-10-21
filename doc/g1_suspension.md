# G1 启动悬挂扭动排查

现场发生在仅打开仿真和控制器、尚未使能/复位/进入策略时。配套命令为 C++ 仿真器
`--parkour --parkour-config example/exported/g1_test/g1_ca_pe_amp_vae_cfg.json`
与 `g1_ca_pe_amp_vae --config ... --model-dir example/exported/g1_test`，domain 1、端口 17200。
因此策略摆腿和已使能 PD 更新时序不能解释这个阶段的持续扭动。

## 直接原因

`simulate/src/parkour_observation.h` 的构造函数原先启动即将全部电机关节的
`dof_damping`、`dof_frictionloss` 清零，以对齐训练中的执行器设置。
但控制器在 `9 → p` 之前输出的 kp/kd/tau 都为零，没有任何电机阻尼接管。
挂带只对 torso 施加平移力，不保持关节角度，也不锁定姿态，于是吊起后的自由关节长期摆动。
跑酷和 CA PE 共用这一仿真类，两者都受影响。

Python `simulate_python/parkour_observation.py` 创建的 `MujocoAdapter` 同样启动即清除被动损耗，
所以其 DDS 仿真路径也有这个问题。独立参考推理 `example/parkour/run_mujoco.py` 每步直接执行
PD，与尚未使能电机的等待阶段不同。

修复保存 MJCF 原有阻尼和摩擦，以实际命令中的非零 kp/kd/tau 判断控制是否启用：
未启用时恢复被动损耗；初始化/策略控制启用时去掉额外损耗，以保持训练动力学；
清零命令后恢复被动损耗。不添加位置保持或额外主动电机力矩。

## 无电机命令对照

使用仓库 C++ MuJoCo、G1 29DOF 平地 MJCF、默认挂带，每次仿真 20 秒，ctrl 始终为零。
统计后 10 秒的各关节峰值速度：

| 动力学与挂带 | 原跑酷 | CA PE（与 g1_test 的惯量/步长/PD 参数相同） |
| --- | ---: | ---: |
| 原逻辑：清除被动损耗、骨盆反馈 | 2.062 rad/s | 1.561 rad/s |
| 仅修正为受力躯干质心反馈，仍清除被动损耗 | 1.705 rad/s | 1.660 rad/s |
| 完整修复：未使能时保留被动损耗，质心反馈 | 0.01069 rad/s | 0.01067 rad/s |

修正挂带反馈本身不能解决未使能时的持续摆动。修复后另对现场 `g1_test` 配置直接运行同一
回归，峰值为 0.01067 rad/s，电机输出始终为零。初始吊起仍有短暂自然摆动，不是刚性锁定。

## 一并发现并修正的问题

- 两端原先用 pelvis 的 qpos/qvel 计算挂带，却把力施加到 torso 质心。腰关节运动时，阻尼没有跟随
  实际受力点。现在使用 `xipos` 和世界系 `mj_objectVelocity(mjOBJ_BODY)` 的线速度，计算和施力点一致。
- C++ 普通模式原先依赖独立 DDS 线程更新 PD，物理补步持有同一互斥锁，可能多步复用旧力矩。
  盲走悬挂保持实验中，2 ms 逐步更新时稳定后峰值约 0.05 rad/s；旧力矩保持 16 ms 时约 56 rad/s。
  现在所有模式均由物理线程每步用当前 qpos/qvel 重算 PD。原 `--parkour` 已逐步更新，
  此缺陷不应作为本次未使能悬挂现象的解释。Python 本来就每物理步更新 PD，没有同一更新时序问题。
- 旧键盘重新挂接会继续拉向原来的世界原点；现在对准当前受力质心重新挂接，避免额外横向拉扯。
  键盘修改通过队列/原子计数交给物理线程，避免与步进并发修改挂带。
- Python 挂点与锚点重合时原先除零，现在返回零力。
- 中英文主 README 原先将 7/8 写反。实际为 7 缩短挂带、吊起，8 放长挂带、放下。

## 验证与操作

项目模块与链路见 [project_structure.md](project_structure.md)。关键入口：
`simulate/src/main.cc` 物理步、`unitree_sdk2_bridge.h` DDS/PD、`parkour_observation.h` 任务动力学，
`example/cpp/g1_visual_controller.h` 两种视觉控制器状态机；模型与配置在 `example/exported/<task>/`。

```bash
cmake -S simulate -B simulate/build
cmake --build simulate/build --target unitree_mujoco -j4
python3 example/cpp/tests/run_tests.py
python3 example/cpp/tests/run_tests.py --test suspension-sim --verbose
python -B simulate_python/test/test_elastic_band.py
```

正式仿真器重新构建通过；6 项 C++ 离线检查、三个任务的零力矩悬挂检查及 5 项 Python 行为检查通过。
最终修复后，`g1_parkour` 与现场 `g1_test` 的正式控制器各通过挂带落地、零速站立、超时停机、
整机重置四项闭环检查，共八项；另通过两项 Python 参考关节映射检查。
闭环检查使用正式 C++ 控制器、本机 UDP/DDS 和临时仿真驱动；不涉及实机。

原启动命令可以继续使用，需重启仿真器加载新二进制。需要关节保持时按 `9 → p`，等待完成；
启动策略前先放下至双脚接地。Y/A 升降不会让视觉策略退出，View/R3 也只清零速度命令；
已进入策略后需要吊起检查应先按 B/p 返回初始化保持。
