# CA PE 行走后仰与 CtrlZ 配置对照（2026-10-20）

两个部署目录都使用 pelvis 状态，但这不代表模拟的机器人、执行器和相机输入相同。
已在本仓库 C++ 闭环复现 `g1_test` 的后仰；核对发现与 CtrlZ 存在有效阻尼、相机姿态及
机器人版本等差异。没有发现默认角、关节方向、PD 配置数组或骨盆重力投影的共同配置错误。
实验说明阻尼和相机差异会改变后仰，但尚未证明某一个差异能单独解释现场的全部现象。

## 对照对象与模型一致性

本次实际读取的外部配置：
`/home/mayh/MYH/Project_code/C++/CtrlZ/example/BitbotSimulation/config_AMP_Depth_VAE_CA.json`。
该路径不在本仓库内；外部仓库只读，没有修改或运行其完整 UI/调度器。

当前 CtrlZ 配置指向训练目录
`2026-10-19_19-42-01_g1_target_depth32*32_flat+stair+slope+gap+platform_CA_ZinCA/exported/policy.onnx`。
它与本仓库 `example/exported/g1_test/exported/policy.onnx` 的 SHA256 相同：
`7eb343b957841cd1334557ad037fa26c35d568e8bb71cbc7ccce3096a1fb27ec`。
因此本次 g1_test 对照排除了模型文件不同这一因素。
较早的 [CtrlZ 对齐记录](g1_ctrlz_alignment.md) 比较的是当时另一条模型路径，不适用于当前配置。

`g1_CA_PE_AMP_VAE` 是另一组网络，不能把它与当前 CtrlZ 配置直接当作同一策略比较。
两个本仓库部署 JSON 的控制、相机、默认角等参数相同，差别为动作历史时序：
旧任务默认 `history_action_delay=1`，g1_test 明确为 0，分别对应各自训练归档的实现。
不应为统一部署而强行改成同一个值。

## 逐项核对

| 项目 | 本仓库 | 当前 CtrlZ | 判断 |
| --- | --- | --- | --- |
| 状态根 | pelvis | pelvis IMU | 一致 |
| 关节顺序与方向 | pelvis 广度优先 → DDS；全部 +1 | 对应同一顺序与正方向 | 未发现反号/错位 |
| 默认角 | hip pitch −0.312、knee 0.669、ankle pitch −0.363、waist pitch 0 | 同值 | 无固定腰部后仰偏置 |
| Kp、Kd、action scale | 保存训练完整精度 | 约六位小数 | 最大差约 5×10⁻⁷，可忽略 |
| 策略周期 | 20 ms | 2 ms × 分频 10 = 20 ms | 一致 |
| 物理周期/积分器 | 5 ms、场景默认积分器 | 2 ms、RK4 | 不同；步长对照未消除后仰 |
| 控制启用时关节阻尼 | 显式 PD 的 Kd，额外被动阻尼为 0 | 设备 position PD 的 Kd，加 XML 被动 damping≈Kd | 有效阻尼不同 |
| 相机原点 | torso 完整姿态旋转 offset | 同样完整姿态旋转 offset | 一致 |
| 相机射线方向 | torso 完整姿态 × offset rotation | torso 仅 yaw × offset rotation | roll/pitch 时深度输入不同 |
| 相机主点 | W/2、H/2 | (W−1)/2、(H−1)/2 | 半像素差异 |
| 自遮挡处理 | 训练视觉 mesh 与地面，逐 geom 过滤近命中 | 组 0+1，近命中时按 body 排除重投 | 不同 |
| 机器人模型 | g1_29dof.xml | g1_29dof_rev_1_0.xml | 不同修订版 |
| 总质量 / torso 合并质量 | 35.112142 / 9.598 kg | 33.340142 / 7.817 kg | 不同 |
| 脚底接触 | 四个球 | 盒体 | 不同 |
| hip roll 力矩上限 | MJCF 88 Nm，限制了 JSON 中的 139 | 139 Nm | 不同 |

两份部署 JSON 均通过 `export_ca_pe_config.py --check`，与各自 params 快照一致。
g1_test 完整 ONNX 与拆分 encoder+actor 的输出对照通过。
使用实际 CtrlZ SO3 模板、实际 MuJoCo pelvis gyro 和 UDP 观测，45 组混合姿态检查中：
角速度误差 7.18×10⁻⁹ rad/s，投影重力及 actor 对应输入误差 1.79×10⁻⁷。
这支持本次后仰并非骨盆/躯干根状态混用或俯仰符号反转。

## 训练相机的关键细节

训练快照写有 `ray_alignment: yaw`，但 `class_type` 是自定义
`NoisyGroupedRayCasterCamera`，继承 `GroupedRayCasterCamera`。
当前可读的 `_update_buffers_impl` 直接将完整父姿态与 offset 组合，再旋转射线，
没有按 ray_alignment 去掉 roll/pitch。噪声子类调用父类后才执行噪声与历史处理。
所以不能仅凭 YAML 字段把本仓库相机判为错误，也不能直接把正式参数改成 CtrlZ 的 yaw。
该判断依据当前可读训练源码，不证明历史 checkpoint 当天的相机文件未变。

## 行走实测与单因素对照

统一使用正式 `g1_ca_pe_amp_vae`、真实 C++ MuJoCo/UDP/ONNX/DDS、无手柄、无挂带的平地行走。
从 0.1 递增至 0.5 m/s，统计 4.5～9 秒，正 pitch 表示前倾，负值表示后仰。
除表格指定项外，保持 g1_test 配置与网络不变。每项为一次受控运行，不能作为统计显著性结论。

| g1_test 条件 | 平均 vx (m/s) | pelvis pitch | torso pitch |
| --- | ---: | ---: | ---: |
| 正式配置 | 0.504 | −6.80° | −6.61° |
| 仅物理步长改为 2 ms | 0.503 | −6.79° | −6.51° |
| 仅替换 torso 合并惯性参数 | 0.506 | −6.72° | −6.88° |
| 仅盒状脚底 | 0.502 | −6.85° | −6.73° |
| 完整 rev_1_0 模型，仍用本仓库执行器/相机 | 0.505 | −6.96° | −7.44° |
| 仅显式 Kd×2，近似考察额外阻尼影响 | 0.607 | −6.18° | −0.27° |
| 仅相机改为 yaw 方向跟随 | 0.513 | −5.42° | −4.25° |

原配置的腰 pitch 平均实际约 0.002 rad，目标约 +0.048 rad；后仰主要是 pelvis 和 torso 一起倾斜，
而非配置给腰部施加了一个固定负角度。
增加阻尼明显改变腰部与整机闭环，但同一 0.5 m/s 命令下实际速度升到 0.607 m/s，
所以不能把 Kd×2 当作已验收的修复。它也不完全等价于 CtrlZ 隐式被动 damping 与显式 PD 的组合。
改用 yaw 相机只部分减轻后仰，且该方式未证实与训练相机匹配。
减轻 torso、换脚底或更换完整机器人版本均未单独消除后仰，因此不能将质量或脚底直接命名为根因。

旧 `g1_CA_PE_AMP_VAE` 在相同命令下平均 vx=0.507 m/s，pelvis pitch≈−4.49°、torso pitch≈−0.43°，
期间 torso 可在约 −10.7° 到 +8.0° 间摆动。它与 g1_test 的平均姿态并不完全一样；
观察单个步态相位不能代替一段行走的平均姿态。

## 本次改动与复现

正式模型、部署 JSON、PD、默认关节角和 ONNX 均未改动。没有添加固定腰角或俯仰补偿。
新增内容为：

- C++ 闭环检查输出每秒 pelvis/torso/waist 俯仰及平均腰部目标，便于区分整机后倾和弯腰。
- C++/Python 支持可选 `camera.ray_alignment: body | yaw`，未指定时保持原 body 行为。
  该参数用于明确相机语义和受控对照，没有写入正式任务配置。
- 实际 UDP 相机与解析平面深度检查，最大误差 5.35×10⁻⁸ m；Python 对应检查通过。

```bash
# 两个任务分别测量；相同 domain 的行走检查不要并行
python3 example/cpp/tests/run_tests.py --test ca-pe-sim --sim-mode walk --verbose
python3 example/cpp/tests/run_tests.py --test ca-pe-sim --sim-mode walk \
  --task-dir example/exported/g1_test --verbose

# 模型输出与观测数学对照
python3 example/cpp/tests/run_tests.py --test ca-pe --test ca-pe-observation \
  --task-dir example/exported/g1_test \
  --ctrlz-root '/home/mayh/MYH/Project_code/C++/CtrlZ' --verbose
```

临时对照均通过统一测试入口构建、运行并清理；对照 JSON/XML 在完成后删除。
`walk` 既有通过标准仅覆盖前进和不倒地，不代表姿态或航向跟踪合格。

下一步应在同一网络、命令、初始状态下记录 CtrlZ 的 pelvis/torso/waist 俯仰和关节目标，
再将执行器阻尼实现与相机输入分别对齐；目前没有完成 CtrlZ 全调度闭环的轨迹对照，
也没有复跑 IsaacLab 原训练环境，不能将剩余后仰判定为策略训练问题。
