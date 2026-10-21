# RobotLab 视觉相机跟随方式：运行时复核（2026-10-21）

结论：当前 rblab 环境中，这些 G1 AMP 视觉任务的相机使用 torso_link 的完整姿态，
包含 roll、pitch、yaw。配置对象仍写着 ray_alignment=yaw，但实际执行的自定义相机方法
不读取这个字段。这里的“完整姿态”指躯干 link，不是直接绑定骨盆根姿态或抽象的“全身”。

## 本次加强的验证

先前执行的是源码提取后的变换函数，并未启动 Isaac Sim；本次进一步：

1. 确认 rblab 实际导入的 robot_lab 为用户指定源码路径，isaaclab 为 IsaacLab-2.3.2。
2. 在无界面 Isaac Sim 中通过真实 Gym 注册项构造五个任务的环境配置。
3. 检查实际相机类 MRO 和 `_update_ray_infos` 函数对象的归属。
4. 创建一个 CPU PhysX 刚体，用真实 RigidBodyView 提供姿态，将实际相机类的射线更新方法
   应用于该 view。未复制更新函数、未替换位姿读取函数、未启动 ONNX 或下发机器人命令。
   相机对象只配置该更新方法需要的数据，未完整初始化相机网格、噪声与图像历史。

本次没有加载完整 G1 训练场景、没有进行 Warp 地形射线求交，也没有重跑训练。
该测试直接验证配置构造、方法分派、PhysX 位姿读取及射线姿态变换，不能代替完整训练回放。

## 实际构造结果

以下五个注册任务均得到：

```text
class_type = NoisyGroupedRayCasterCamera
prim_path = {ENV_REGEX_NS}/Robot/torso_link
ray_alignment = yaw
attach_yaw_only = None
```

- RobotLab-Isaac-AMP-Unitree-G1-Walk-VAE-Sensor-v0
- RobotLab-Isaac-AMP-Unitree-G1-Walk-VAE-Sensor-CA-v0
- RobotLab-Isaac-AMP-Unitree-G1-Walk-VAE-Sensor-Target-CrossAttention-v0
- RobotLab-Isaac-AMP-Unitree-G1-Walk-VAE-Sensor-Target-CASA-v0
- RobotLab-Isaac-AMP-Unitree-G1-Walk-VAE-Sensor-Target-GRU-CA-v0

## 为什么普通 RayCaster 的 yaw 分支没有生效

实际类 MRO 为：

```text
NoisyGroupedRayCasterCamera
 → NoisyCameraMixin
 → GroupedRayCasterCamera
 → RayCasterCamera
 → GroupedRayCaster
 → MultiMeshRayCaster
 → RayCaster
 → SensorBase
```

NoisyGroupedRayCasterCamera 的 `_update_buffers_impl` 调用 super；NoisyCameraMixin 没有实现
这个方法，故进入 GroupedRayCasterCamera。后者调用 `self._update_ray_infos`，实际函数对象
归属也是 GroupedRayCasterCamera，没有进入 RayCaster 中读取 ray_alignment 的方法。

该函数直接执行：

```text
(p_torso, q_torso) = obtain_world_pose_from_view(view)
p_camera = p_torso + R(q_torso) * offset_pos
q_camera = q_torso * offset_quat
ray_world = R(q_camera) * ray_local
```

PhysX 位姿读取只读取 get_transforms 并转换四元数排列，不会去掉 roll/pitch。
相机安装外参也不会抵消动态躯干姿态；Target 的 reset 随机化只是重新扰动安装外参。

## 实际 PhysX 刚体输入结果

安装四元数使用真实任务配置，标称光轴向下约 48°：

| 输入刚体 pitch | 实际相机光轴俯角 |
| --- | ---: |
| 0° | 47.9991958° |
| +10° | 57.9990227° |
| −10° | 37.9993193° |

每个姿态下将 ray_alignment 分别设为 yaw、base、world，射线结果相同，数值断言通过。
这同时排除了“父类先把姿态处理成 yaw，再交给自定义方法”的解释。

## 对前述解释的更正

GroupedRayCasterCameraCfg 源码虽然存在 `attach_yaw_only=False` 的 post-init，但这些任务的
实际配置值是 None。NoisyGroupedRayCasterCameraCfg 的多继承首先解析到 NoisyCameraCfgMixin
生成的 post-init，不能把第二个配置基类的那行赋值当作实际配置输出。
因此，不能用那行 False 作为 yaw-only 被关闭的运行时证据。

最终结论依据实际方法分派、完整四元数读取、变换公式与 PhysX 刚体测试。
没有修改训练源码或配置。数值断言全部通过后，Isaac Sim 的 app.close() 阶段未及时返回，
已定向结束本次唯一临时探针进程；不能将这次探针描述为正常退出的完整仿真测试。
一次性验证脚本和日志在验证完成后清理。
