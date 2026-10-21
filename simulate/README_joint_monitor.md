# G1 C++ 网页关节监控

在仓库根目录编译并运行：

```bash
cmake -S simulate -B simulate/build
cmake --build simulate/build --target unitree_mujoco joint_monitor -j4

# 仿真器和控制器照常运行，另开终端启动监控
./simulate/build/joint_monitor --domain 1 --interface lo --port 8765
```

浏览器打开 **http://127.0.0.1:8765**。适用于 G1 29DOF 的 C++ 仿真及三个 G1 控制器；
domain 和网卡应与仿真器一致，不需要 `--parkour`。监控程序没有 Python 运行依赖，
不发布任何 DDS 命令，单独退出不会停止仿真。Ctrl+C 退出监控。

默认只监听本机；需从其他机器查看时，可显式传入 `--host 0.0.0.0`，
并在浏览器使用仿真主机的 IP。其他选项：`--rate 50`（采样与推送上限，支持 0～1000 Hz 之间的正值）、
`--port`、`--domain`、`--interface`，运行 `--help` 查看。

## 页面和数据含义

- 29 个关节按 DDS 电机编号显示角度、速度、`tau_est`、目标、误差和 Kp/Kd。
- 支持关节搜索、弧度/角度切换、暂停显示、多关节勾选及最近 20 秒曲线。
- 显示 LowState 的 IMU 四元数、RPY、角速度和原始加速度。RPY 由 w,x,y,z 四元数归一化计算；
  这里是 LowState IMU，不是 `rt/secondary` 的独立胸部 IMU，也不是策略 UDP 状态。
- Enable 是控制器发布的命令输出开关，不代表物理上电反馈；没有独立使能话题时显示 `—`。
- 独立目标话题优先，即使未使能也可查看目标。目标过期后回退到新鲜的 LowCmd。
  目标、命令和使能的有效期均为 500 ms；无有效目标时页面显示 `—`。
- “监控时间”是监控程序启动后的单调时钟秒数，与 Python 版一致，不是 MuJoCo 仿真时间。
  数据年龄表示距最近一次有效关节采样的墙钟时间；无 DDS 数据时页面等待，停止更新时会提示过期。

## 实现与 Python 版的对应关系

Python 版 `simulate_python/joint_monitor.py` 是独立 DDS 订阅服务，而非 Python 仿真器内部功能。
C++ 版采用相同结构及接口：

| DDS 话题 | 用途 |
| --- | --- |
| `rt/lowstate` | G1 HG LowState，实际关节状态和 IMU |
| `rt/lowcmd` | HG LowCmd，实际命令和目标回退 |
| `rt/g1/reset_target` | HG LowCmd，独立目标和增益 |
| `rt/g1/reset_enable` | HG LowCmd，前 29 个 motor mode 表示使能 |

`joint_monitor_state.h` 负责数据校验、采样限频、目标优先级和 JSON 快照；
`joint_monitor_http.h` 使用非阻塞 HTTP 事件循环，提供 `/`、`/index.html`、
`/api/joints` 和 SSE `/api/stream`。浏览器通过 EventSource 接收状态，按屏幕刷新率绘制。
最多同时处理 32 个客户端，每个 SSE 连接只排队一份快照，慢连接不阻塞 DDS 回调和其他客户端。

前端直接复用 `simulate_python/web/joint_monitor.html`，由 CMake 嵌入 C++ 可执行文件，
运行时不需要源文件或静态文件目录。从任意工作目录启动均可。
前端源文件修改后，重新执行构建会自动更新内嵌页面。

## 验证

先构建正式 `joint_monitor`，再运行：

```bash
python3 example/cpp/tests/run_tests.py --test joint-monitor --verbose
```

检查目标优先级、命令回退、使能和目标过期、IMU、DDS 编号、限频、非法状态拒绝，
以及实际 DDS 发布到正式监控程序后的 HTTP JSON、两个并发 SSE 连接、路由、数据年龄和带 SSE 连接退出。
测试使用临时构建目录并自动清理；不向正式构建加入测试目标。
