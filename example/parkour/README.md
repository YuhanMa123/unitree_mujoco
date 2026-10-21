# G1 Parkour 双网络推理

主部署入口为 **C++ Unitree MuJoCo 采集同步状态/深度，C++ 推理并通过 DDS 控制**。
编译、启动和按键操作见 [C++ 仿真与推理](../cpp/README.md#g1-parkour)。
下方Python推理和独立rollout仅作离线参考及诊断。

默认使用仓库 `example/exported/g1_parkour/` 内的 `0-depth_encoder.onnx` 和 `actor.onnx`，提供独立 ONNX Runtime CPU
推理模块及 MuJoCo 控制入口。无 Isaac Sim 依赖，不覆盖`example/exported/g1_locomotion/policy.onnx`，不发布 DDS 命令。

## Python参考推理与独立诊断（可选）

本机已有包含 NumPy、ONNX Runtime、MuJoCo 的环境：

```bash
cd /home/wsy/rl/unitree_mujoco
# 仅验证双网络，不加载仿真
/home/wsy/miniconda3/envs/instinctlab/bin/python example/parkour/run_mujoco.py --check-policy
# 固定基座，验证关节映射、相机、PD 和完整推理链
/home/wsy/miniconda3/envs/instinctlab/bin/python example/parkour/run_mujoco.py --headless --fixed-base --seconds 1
# 显示 MuJoCo 窗口，零速命令；关闭窗口或 Ctrl+C 结束
/home/wsy/miniconda3/envs/instinctlab/bin/python example/parkour/run_mujoco.py --seconds 10
# 指定场景、模型目录、三维命令
/home/wsy/miniconda3/envs/instinctlab/bin/python example/parkour/run_mujoco.py \
  --model-dir example/exported/g1_parkour \
  --scene unitree_robots/g1/scene_29dof.xml --command 0.3 0 0 --seconds 10
# 自动检查
/home/wsy/miniconda3/envs/instinctlab/bin/python -m unittest discover -s example/parkour/tests -v
```

其他环境可安装 `requirements.txt`。`--config` 可指定替代 `g1_torsobase_cfg.json`。
无界面模式按仿真时间运行；窗口模式按物理步进行实时节流。
独立运行器自行创建仿真，无需启动 `simulate_python/unitree_mujoco.py`。

## 模块接口

```python
from policy import ParkourPolicy
policy = ParkourPolicy(model_dir='/path/to/exported')
# 每 0.02 秒调用一次；所有关节数组必须按 policy.joint_names 排列
q_target = policy.step(q, dq, torso_gyro, projected_gravity, [vx, vy, yaw_rate], depth_m)
# reset() 清空历史和上一动作，每次重新进入策略时调用
```

`torso_gyro` 为 torso 机体系角速度，`projected_gravity` 为世界 `[0,0,-1]`
在 torso 机体系的方向。不能直接使用 pelvis IMU 代替 torso 状态。
`depth_m` 是 `36×64` 图像平面深度，单位米，上方为首行，左方为首列；
不能传沿射线距离。无命中用正无穷，负值和 NaN 拒绝接收。
数据采集与时间同步由调用方负责，本接口假定每次调用收到同一控制时刻的新状态和深度。

推理流水线：

1. 每个本体观测项各保留 8 帧，由旧到新展开，再按项拼接：角速度×0.25、
   重力、三维速度命令、关节位置减默认角、关节速度×0.05、上一原始动作。
   共 `(3+3+3+29+29+29)×8 = 768` 维。
2. 深度裁去上方18行、左右各16列，得到 `18×32`；3×3、sigma=1 的反射边界
   Gaussian 模糊，裁剪到 `[0,2.5]` 米后除以2.5。无额外 resize。
3. 深度缓存37帧，抽取距当前35、30、25、20、15、10、5、0帧的8张图。
   `depth_delay_frames` 支持0或1，默认0，不随机注入训练延迟。
4. 编码器 `[1,8,18,32] → [1,128]`；将768维历史与128维特征拼接，
   动作网络 `[1,896] → [1,29]`。
5. 训练坐标目标角为 `default + action_scale × raw_action`；反馈原始动作，不额外裁剪。
   PD 每0.002秒更新，按训练力矩限制和当前模型电机限制的较小值限幅。

初始历史用第一份有效观测填充。模型形状、输入有限性、关节电机映射均在运行时检查。
MuJoCo 相机通过 CPU 射线生成深度，无需 OpenGL 渲染。相机使用训练视场角、torso偏移
和完整torso旋转，将射线距离投影为图像平面深度；仅查询模型 group 0 的碰撞几何。

## 配置来源与验证边界

导出目录只有两个网络，没有此次训练的 `params/env.yaml` 或关节名称元数据。
`g1_torsobase_cfg.json` 据本机 InstinctLab 下列源码重建，不能证明源码与当时训练完全一致：

- `tasks/parkour/config/parkour_env_cfg.py`：观测、深度、周期和动作缩放。
- `tasks/parkour/config/g1/g1_parkour_target_amp_cfg.py`：torso根模型及带鞋配置。
- `assets/unitree_g1.py`：文件头记载的torso根关节顺序、默认角和BeyondMimic PD参数。

训练模型带鞋、以torso为根；当前仓库MuJoCo模型以pelvis为根，几何、惯量、碰撞和相机
射线遮挡模型均存在差异。运行器按关节名称映射，并读取torso姿态和速度；
腰部三个关节因父子链反转，使用 `joint_sign=-1` 转换q、dq和目标，其余为+1。没有重建完整
训练机器人，也没有模拟训练中的随机电机延迟。训练相机近距裁剪和网格遮挡与MuJoCo
碰撞几何射线也未做逐像素等价验证。需要稳定运动时，应先核对训练时实际关节顺序和
参数，再对齐带鞋模型的动力学及传感器。

已验证（未使用Isaac Sim）：

- 双网络CPU推理通过，默认观测目标角范围约 `[-0.5956, 0.7202]` rad。
- 6项Python自动检查通过：历史/深度抽帧、预处理、网络拼接/动作反馈、非法输入，
  以及腰链正向运动学和关节速度/力矩方向回归检查。
- 固定基座MuJoCo运行1秒、500物理步通过。
- 修复腰部方向后，独立诊断运行零命令10秒，最低torso高度0.806m，结束时XY位移0.075m。
- 此前 Python Unitree 仿真→C++ 推理→DDS 控制链路的零命令落地检查已通过。
  跑酷尚未验证；保护在torso低于0.5m或倾斜超过1rad时停止。
  此前自由落地失败是方向转换遗漏，旧的固定基座检查没有覆盖物理坐标含义。

模型SHA256：

```text
0-depth_encoder.onnx b184217a93d3e54a2cf79b786be71f296bccea061e20cfcd454d9877f0633b8e
actor.onnx           24119b88e71394bbfb0cfa68479bded782d6fe016f132f79b3d3d3b61880343b
```
