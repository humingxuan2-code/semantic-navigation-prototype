# 语义导航机器人仿真项目日志

> 项目名称：Semantic Navigation Prototype  
> 当前阶段：Gazebo Fortress 差速车控制与 odometry 闭环控制  
> 工作目录：`~/semantic_nav_ws`

---

## 一、项目目标

本项目在 Windows 的 WSL2 Ubuntu 环境中，使用 ROS 2 Humble 和 Gazebo Fortress 搭建移动机器人仿真环境，并逐步实现：

- 差速移动机器人的基础控制；
- 基于 Gazebo Transport 的可靠速度控制；
- 基于 odometry 的位置与朝向反馈；
- 闭环距离控制；
- 闭环转向控制；
- 指定坐标点导航；
- 后续扩展到轨迹记录、地图、语义导航与具身智能任务。

---

## 二、当前环境

| 项目 | 配置 |
|---|---|
| 宿主系统 | Windows |
| Linux 环境 | WSL2 |
| Ubuntu | Ubuntu 22.04.5 LTS |
| Linux 用户 | `openclaw` |
| ROS 版本 | ROS 2 Humble |
| 仿真器 | Gazebo Fortress / Ignition Gazebo |
| 编程语言 | C++17 |
| Python | Python 3.10.12 |
| GPU | NVIDIA GeForce RTX 3060 Laptop GPU |
| GPU 状态 | WSL 中 `nvidia-smi` 可正常识别 |
| 工作目录 | `~/semantic_nav_ws` |

关键工具与库：

```text
build-essential
pkg-config
colcon
ignition-transport11
ignition-msgs8
```

---

## 三、环境搭建与踩坑记录

### 3.1 WSL2 与 Ubuntu

已完成：

- WSL2 正常运行；
- Ubuntu 22.04.5 LTS 可正常启动；
- 当前用户目录为 `/home/openclaw`；
- Python 3.10 可用；
- NVIDIA RTX 3060 可在 WSL 中通过 `nvidia-smi` 识别。

常用验证命令：

```bash
pwd
python3 --version
nvidia-smi
```

### 3.2 网络、VPN 与 apt 软件源

曾遇到的问题：

- 使用新加坡 VPN 时下载 Ubuntu 软件包很慢；
- `apt update` 曾卡在 `deb.nodesource.com`；
- 默认 Ubuntu 软件源下载 ROS / Gazebo 依赖较慢；
- ROS 与 Gazebo 首次安装依赖多、耗时长。

最终处理：

- 使用中国 USTC 镜像源；
- 定位并禁用了卡住更新的 NodeSource source；
- 后续 `apt update` 与 ROS / Gazebo 包下载恢复正常。

经验：

> `apt update` 长时间卡在某一个第三方源时，应先定位并禁用该源，而不是持续等待或无差别重试。

---

## 四、ROS 2 基础验证

### EXP-001：Talker / Listener 通信

目标：验证 ROS 2 基础发布订阅通信。

命令：

```bash
# 终端 1
ros2 run demo_nodes_py talker

# 终端 2
ros2 run demo_nodes_py listener
```

结果：

- Talker 持续发布 `Hello World`；
- Listener 成功接收消息；
- ROS 2 基础发布订阅正常。

结论：**通过**

备注：Listener 没有输出时，通常是 Talker 没有在另一终端同时运行。

---

## 五、RViz、Gazebo Classic 与 TurtleBot3 尝试

### 5.1 RViz

RViz 能正常打开。

曾显示：

```text
Fixed Frame: No tf data
```

解释：

- RViz 程序本身正常；
- 当时没有 TF、地图或机器人数据；
- 这是预期提示，不代表 RViz 安装失败。

### 5.2 Gazebo Classic / TurtleBot3

尝试过：

- TurtleBot3 相关包；
- Gazebo Classic；
- TurtleBot3 spawn；
- ROS 2 与 Gazebo bridge。

主要问题：

- WSL 下 Gazebo Classic GUI 黑屏或渲染不稳定；
- TurtleBot3 生成过程曾失败；
- `gzserver` / spawn service 启动不稳定；
- ROS CLI 与 Gazebo bridge 的跨终端发现不可靠。

技术决策：

> 不继续把时间投入 Gazebo Classic / TurtleBot3 Classic 的重复排错；转向在当前 WSL 环境中更稳定的 Gazebo Fortress / Ignition Gazebo。

---

## 六、Gazebo Fortress 差速车仿真

成功启动命令：

```bash
source /opt/ros/humble/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch ros_gz_sim_demos diff_drive.launch.py rviz:=false
```

仿真世界包含：

```text
vehicle_blue
vehicle_green
ground_plane
sun
```

蓝车关键 topic：

```text
速度控制：/model/vehicle_blue/cmd_vel
里程计：  /model/vehicle_blue/odometry
```

验证结果：

- Gazebo Fortress GUI 能显示世界、蓝车和绿车；
- `vehicle_blue` 的差速控制器可正常工作；
- 仿真可播放、暂停及重启。

---

## 七、ROS-Gazebo Bridge 问题与当前路线

`parameter_bridge` 可随 demo 启动，但 ROS CLI 在其他终端无法稳定发现 bridge topic。

典型现象：

```bash
ros2 topic list --no-daemon
```

多次只显示：

```text
/parameter_events
/rosout
```

已尝试：

- `ros2 daemon stop`；
- 使用 `--no-daemon`；
- 对比 CLI、bridge 进程环境变量；
- 调整 `ROS_DOMAIN_ID`；
- 调整 `ROS_LOCALHOST_ONLY`；
- 设置 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`；
- 重启 Gazebo、bridge、终端及 WSL；
- 检查 bridge / Gazebo 进程状态。

结论：

- Gazebo、机器人模型、差速控制 topic 均正常；
- 当前 WSL 环境中，ROS CLI 与 `ros_gz_bridge` 的跨终端发现不稳定；
- ROS topic 控制无法作为当前项目的可靠路径；
- 暂时不再反复尝试同一批 bridge 配置。

当前路线：

```text
直接使用 Gazebo Transport
↓
绕开不稳定的 ROS-Gazebo bridge
↓
继续完成控制、odometry 与导航逻辑
```

---

## 八、Gazebo Transport 控制验证

验证命令：

```bash
ign topic -t /model/vehicle_blue/cmd_vel \
  -m ignition.msgs.Twist \
  -p "linear: {x: 0.30}, angular: {z: 0.0}"
```

验证结果：

```text
Gazebo Transport 指令
→ Gazebo DiffDrive 控制插件
→ vehicle_blue 运动
```

这证明以下链路已通：

```text
速度消息发布
→ /model/vehicle_blue/cmd_vel
→ 差速驱动控制器
→ 蓝车实际运动
```

---

## 九、Bash 短命令控制失败记录

早期尝试通过 Bash 脚本及短时间 `ign topic` 发布实现：

```text
前进一段时间
→ 发送零速度
→ 自动停止
```

实际问题：

- Shell 脚本结束后，终端会回到绿色提示符；
- 但蓝车有时仍保留前一次非零速度，继续向前开；
- 短命令或单次零速度消息无法稳定覆盖前进指令；
- 蓝车可能越开越远。

旧脚本：

```text
~/semantic_nav_ws/tools/drive_blue.sh
```

结论：

> 旧 `drive_blue.sh` 不再使用。后续一律使用 C++ 持久 Gazebo Transport publisher，并在动作结束后连续发送零速度。

---

## 十、C++ Gazebo Transport 控制器

### 10.1 编译依赖

已确认：

```bash
sudo apt install -y build-essential pkg-config
```

实际可用库：

```text
ignition-transport11
ignition-msgs8
```

Python binding 检查结果：

```text
Python 中不存在 ignition.transport 模块
```

因此不走 Python Transport binding，采用 C++。

### 10.2 基础控制器

源码：

```text
~/semantic_nav_ws/tools/drive_blue.cpp
```

可执行文件：

```text
~/semantic_nav_ws/tools/drive_blue_cpp
```

编译命令：

```bash
g++ -std=c++17 -O2 \
  ~/semantic_nav_ws/tools/drive_blue.cpp \
  -o ~/semantic_nav_ws/tools/drive_blue_cpp \
  $(pkg-config --cflags --libs ignition-transport11 ignition-msgs8)
```

可靠停止的关键设计：

```text
创建持久 publisher
→ 以 20 Hz 连续发布动作消息
→ 动作完成后连续发布 2 秒零速度
→ 再退出程序
```

已验证命令：

```bash
~/semantic_nav_ws/tools/drive_blue_cpp forward
~/semantic_nav_ws/tools/drive_blue_cpp left
~/semantic_nav_ws/tools/drive_blue_cpp right
~/semantic_nav_ws/tools/drive_blue_cpp route
~/semantic_nav_ws/tools/drive_blue_cpp stop
```

| 命令 | 动作 | 结果 |
|---|---|---|
| `forward` | 前进约 3 秒 | 成功，并可靠停止 |
| `left` | 原地左转约 90° | 成功，并可靠停止 |
| `right` | 原地右转约 90° | 成功，并可靠停止 |
| `route` | 固定 L 形路线 | 成功，并可靠停止 |
| `stop` | 连续发送零速度 | 成功停止 |

---

## 十一、Odometry 验证

查看蓝车里程计：

```bash
ign topic -e -t /model/vehicle_blue/odometry
```

已观察到持续更新的字段：

```text
position:
  x: ...
  y: ...

orientation:
  z: ...
  w: ...
```

验证结论：

- 可实时读取 `vehicle_blue` 的 x、y 位置；
- 可读取四元数 orientation；
- 后续可由四元数计算 yaw；
- 位置和朝向都可以作为闭环控制反馈。

---

## 十二、实验记录

### EXP-002：固定时间前进与可靠停止

目标：让蓝车前进后稳定停止。

方法：

```text
20 Hz 连续发布前进速度
→ 前进约 3 秒
→ 连续 2 秒发布零速度
```

结果：

- 蓝车成功前进；
- 蓝车在动作结束后可靠停止。

结论：**通过**

---

### EXP-003：原地左转与右转

目标：只改变朝向，不产生明显前进位移。

方法：

```text
left：
linear.x = 0
angular.z = +1.0

right：
linear.x = 0
angular.z = -1.0
```

结果：

- 蓝车能原地左转约 90°；
- 蓝车能原地右转约 90°；
- 动作结束后可靠停止。

结论：**通过**

---

### EXP-004：固定 L 形路线

路线：

```text
前进
→ 原地左转
→ 再前进
→ 最终停止
```

结果：

- 蓝车完成固定 L 形路线；
- 最后稳定停住。

结论：**通过**

---

### EXP-005：基础 odometry 闭环距离控制

源码：

```text
~/semantic_nav_ws/tools/drive_blue_distance.cpp
```

目标：依据 odometry 实际位移，让蓝车前进 0.500 m。

方法：

```text
订阅 odometry
→ 记录起点 x、y
→ 固定速度前进
→ 欧氏位移达到目标时停止
```

结果：

```text
目标距离：0.500 m
实际位移：0.600 m
绝对误差：0.100 m
```

分析：

- 已经实现“按实际位移停止”，不再只依赖固定时间；
- 但接近目标仍使用固定速度，存在明显超调。

结论：**通过，但需要减速优化。**

---

### EXP-006：比例减速闭环距离控制

源码：

```text
~/semantic_nav_ws/tools/drive_blue_distance_slow.cpp
```

可执行文件：

```text
~/semantic_nav_ws/tools/drive_blue_distance_slow
```

测试命令：

```bash
~/semantic_nav_ws/tools/drive_blue_distance_slow 0.50
```

目标：

- 让蓝车前进 0.500 m；
- 接近目标时自动减速；
- 降低最终距离误差。

核心参数：

```text
最高速度：0.18 m/s
最低速度：0.025 m/s
比例增益：0.45
停止阈值：0.012 m
控制频率：20 Hz
```

实际结果：

```text
目标距离：0.500 m
起点：x=0.641，y=1.025
终点：x=0.735，y=1.513
实际位移：0.498 m
绝对误差：0.002 m
相对误差：约 0.4%
```

过程特征：

```text
远离目标：较快前进
接近目标：逐步降速
进入停止阈值：连续发布零速度
最终：稳定停止
```

对比：

| 版本 | 目标距离 | 实际位移 | 绝对误差 |
|---|---:|---:|---:|
| 基础固定速度版本 | 0.500 m | 0.600 m | 0.100 m |
| 比例减速版本 | 0.500 m | 0.498 m | 0.002 m |

结论：**通过。比例减速显著降低了超调。**

---

### EXP-007：基于 yaw 的闭环 90 度转向

源码：

    ~/semantic_nav_ws/tools/drive_blue_yaw.cpp

可执行文件：

    ~/semantic_nav_ws/tools/drive_blue_yaw

目标：

- 从 `/model/vehicle_blue/odometry` 读取 orientation 四元数；
- 将四元数转换为 yaw；
- 根据当前 yaw 与目标 yaw 的误差实时调整角速度；
- 实现闭环左转和右转约 90 度；
- 转向完成后稳定停车。

测试命令：

    ~/semantic_nav_ws/tools/drive_blue_yaw left90
    ~/semantic_nav_ws/tools/drive_blue_yaw right90

核心控制参数：

- 最大角速度：0.80 rad/s
- 最小角速度：0.08 rad/s
- 比例增益：1.50
- 停止阈值：0.020 rad，约 1.15 度
- 控制频率：20 Hz
- 停止方式：连续发送 2 秒零速度

左转实验结果：

- 实际转角：90.40 度
- 最终角度误差：0.40 度

右转实验结果：

- 起始 yaw：169.53 度
- 目标 yaw：79.53 度
- 最终 yaw：80.08 度
- 实际转角：-89.44 度
- 最终角度误差：0.56 度

结论：

通过。

蓝车能够依据 odometry 的实时朝向完成闭环约 90 度左转和右转。
与固定时间转向相比，当前控制方式能够读取实际 yaw，并根据角度误差自动减速和停车。

---

### EXP-008：基于位置与 yaw 的闭环目标点导航

源码：

    ~/semantic_nav_ws/tools/drive_blue_goto.cpp

可执行文件：

    ~/semantic_nav_ws/tools/drive_blue_goto

目标：

- 将闭环 yaw 转向与闭环距离控制组合起来；
- 根据蓝车当前的位置和朝向，计算目标点的世界坐标；
- 根据当前朝向和目标方向的误差先进行原地转向；
- 对准目标后向前移动；
- 在运动过程中不断重新读取 odometry，对朝向和位置进行修正；
- 进入目标点附近后稳定停车。

测试命令：

    ~/semantic_nav_ws/tools/drive_blue_goto rel 0.40 0.30

该命令含义：

- 从蓝车当前姿态出发；
- 目标点位于蓝车当前“前方 0.40 m、左侧 0.30 m”的位置；
- 程序自动将相对目标转换为 Gazebo 世界坐标；
- 然后执行“转向 → 前进 → 重新修正 → 停车”。

核心控制参数：

- 目标点停止阈值：0.025 m
- 朝向允许前进阈值：0.035 rad，约 2 度
- 最大线速度：0.18 m/s
- 最小线速度：0.025 m/s
- 距离比例增益：0.45
- 最大角速度：0.80 rad/s
- 最小角速度：0.08 rad/s
- 角度比例增益：1.50
- 控制频率：20 Hz
- 停车方式：连续发送 2 秒零速度

实验结果：

- 起点：x=0.735，y=1.513，yaw=80.085 度
- 自动计算的目标点：x=0.509，y=1.959
- 初始目标距离：0.500 m
- 终点：x=0.509，y=1.956，yaw=120.037 度
- 最终目标误差：0.002 m

运动过程：

- 初始阶段朝向误差约为 36.870 度，机器人先原地转向；
- 朝向误差进入允许范围后，机器人开始前进；
- 接近目标时，机器人根据 odometry 继续进行小幅转向与位置修正；
- 最终进入目标点距离阈值并稳定停车。

结论：

通过。

蓝车已能够基于 odometry 中的实时 x、y 和 yaw 信息，
自动完成相对目标点的世界坐标换算、闭环转向、闭环前进和最终停车。

在“前方 0.40 m、左侧 0.30 m”的测试中，
最终目标误差为 0.002 m，说明基础点到点闭环导航已成功实现。

---

### EXP-009：导航轨迹记录与可视化

源码：

    ~/semantic_nav_ws/tools/drive_blue_goto_trace.cpp

可执行文件：

    ~/semantic_nav_ws/tools/drive_blue_goto_trace

轨迹绘图脚本：

    ~/semantic_nav_ws/tools/plot_trajectory.py

测试命令：

    ~/semantic_nav_ws/tools/drive_blue_goto_trace rel 0.40 0.30

实验目标：

- 在点到点闭环导航过程中持续记录机器人状态；
- 将时间、位置、yaw、目标距离、朝向误差、线速度、角速度与控制模式保存为 CSV；
- 使用 Python 将实际轨迹、起点、目标点和终点绘制为 PNG 图像；
- 为后续多组实验和误差对比提供可复现的数据记录。

CSV 文件：

    ~/semantic_nav_ws/tools/trajectory_latest.csv

轨迹图：

    ~/semantic_nav_ws/tools/trajectory_latest.png

归档目录：

    ~/semantic_nav_ws/outputs/exp009_rel_040_030/

CSV 记录字段：

- time_sec
- x
- y
- yaw_deg
- target_x
- target_y
- distance_to_goal
- heading_error_deg
- linear_speed
- angular_speed
- mode

本次实验结果：

- 起点：x=0.509，y=1.956
- 目标点：x=0.049，y=2.153
- 终点：x=0.052，y=2.152
- 最终目标误差：0.003 m
- 轨迹采样点数量：331
- 生成轨迹图：trajectory_latest.png

轨迹分析：

- 机器人先在原地进行朝向调整；
- 朝向误差进入允许范围后开始前进；
- 运动过程中根据 odometry 不断修正位置和朝向；
- 终点与目标点在轨迹图中几乎重合；
- 最终位置误差约为 3 mm。

结论：

通过。

系统已能够在完成闭环点到点导航的同时，
保存完整的运行轨迹和控制状态，
并自动生成“实际路径、起点、目标点、终点”的可视化结果。

---

### EXP-010：多方向、多距离连续导航基准测试

批量测试脚本：

    ~/semantic_nav_ws/tools/run_navigation_benchmark.sh

汇总分析脚本：

    ~/semantic_nav_ws/tools/summarize_benchmark.py

实验结果目录：

    ~/semantic_nav_ws/outputs/benchmark_rel_v1/

测试方式：

- 使用 rel 相对坐标模式；
- 每一组目标相对于蓝车当前的位置和朝向计算；
- 六组实验连续执行，不在每一组之间重置机器人；
- 每组分别保存终端输出、轨迹 CSV 和轨迹 PNG；
- 最后生成误差汇总表、误差柱状图和轨迹总览图。

测试组别：

1. 前方 0.30 m，左侧 0.00 m
2. 前方 0.30 m，左侧 0.20 m
3. 前方 0.30 m，右侧 0.20 m
4. 前方 0.50 m，左侧 0.00 m
5. 前方 0.40 m，左侧 0.30 m
6. 前方 0.40 m，右侧 0.30 m

各组最终误差：

- case01_straight_030：0.004970 m
- case02_left_030_020：0.008452 m
- case03_right_030_020：0.008885 m
- case04_straight_050：0.005288 m
- case05_left_040_030：0.003785 m
- case06_right_040_030：0.005024 m

总体指标：

- 平均最终误差：0.006067 m，约 6.07 mm
- 最大最终误差：0.008885 m，约 8.88 mm
- 最小最终误差：0.003785 m，约 3.79 mm
- 六组均成功进入 0.025 m 的目标点停止阈值
- 总轨迹采样点数量：1435

生成文件：

    ~/semantic_nav_ws/outputs/benchmark_rel_v1/summary/benchmark_summary.csv
    ~/semantic_nav_ws/outputs/benchmark_rel_v1/summary/final_error_comparison.png
    ~/semantic_nav_ws/outputs/benchmark_rel_v1/summary/trajectory_overlay.png

实验分析：

- 直线目标耗时较短，因为无需进行明显的原地朝向校正；
- 左右侧目标会先进入转向模式，因此整体耗时更长；
- 六组最终误差均低于 1 cm，表现出稳定的闭环收敛能力；
- 轨迹总览图呈连续折线，是因为六组实验连续进行，后一组从前一组终点开始；
- 该实验验证了连续相对目标场景下的导航稳定性，但尚未构成固定初始位姿下的独立重复实验。

结论：

通过。

闭环导航控制器已在六组不同相对方向和距离的目标任务中稳定完成导航。
所有实验的最终误差均小于 1 cm，平均误差约为 6.07 mm。

---

### EXP-011：多航点路线导航与全局轨迹可视化

实验目标：

- 在一次连续任务中依次执行多个相对航点；
- 复用已验证的闭环单点导航控制器；
- 为每个航点保存独立轨迹、终端输出和最终误差；
- 将多个航点的轨迹拼接为完整路线总览图；
- 验证控制器在连续任务中的稳定性。

路线执行脚本：

    ~/semantic_nav_ws/tools/run_waypoint_route.sh

路线汇总脚本：

    ~/semantic_nav_ws/tools/summarize_waypoint_route.py

实验路线：

1. WP1：前方 0.35 m，左侧 0.00 m
2. WP2：前方 0.00 m，左侧 0.35 m
3. WP3：前方 0.35 m，左侧 0.00 m
4. WP4：前方 0.00 m，右侧 0.35 m

实验结果目录：

    ~/semantic_nav_ws/outputs/exp011_waypoint_route_v1/

每个航点保存：

    trajectory.csv
    trajectory.png
    terminal_output.txt
    plot_output.txt

汇总文件：

    ~/semantic_nav_ws/outputs/exp011_waypoint_route_v1/summary/waypoint_summary.csv
    ~/semantic_nav_ws/outputs/exp011_waypoint_route_v1/summary/waypoint_error_comparison.png
    ~/semantic_nav_ws/outputs/exp011_waypoint_route_v1/summary/waypoint_route_overview.png

结果分析：

- WP1 最终误差约为 10.5 mm；
- WP2 最终误差约为 5.7 mm；
- WP3 最终误差约为 10.8 mm；
- WP4 最终误差约为 8.0 mm；
- 四个航点均进入 25 mm 的目标停止阈值；
- 多航点路线图显示机器人依次完成右移、上移、继续前进和右移任务；
- 每段导航均基于 odometry 反馈进行朝向修正、前进和最终停车。

结论：

通过。

系统已能够将闭环单点导航控制器用于连续多航点任务，
并在每个航点自动完成目标坐标计算、朝向调整、位置修正和稳定停车。
同时，系统能够保存分段轨迹并生成多航点路线总览图，
为后续航点序列规划、障碍物地图和语义导航扩展提供基础。

---

---

### EXP-012：CSV 驱动的固定 Odom 坐标多航点路线执行

实验目标：

- 从 CSV 路线文件读取固定 `vehicle_blue/odom` 坐标系航点；
- 在单个 C++ 控制器进程中连续执行整条航点路线；
- 基于 odometry 的 x、y 和 yaw 反馈完成转向、前进、朝向修正和停车；
- 输出统一轨迹 CSV、航点汇总 CSV、路线图和误差图；
- 验证控制器在固定 odometry 坐标航点序列中的稳定性。

路线文件：

`~/semantic_nav_ws/routes/exp012_global_rectangle.csv`

路线控制器：

`~/semantic_nav_ws/tools/drive_blue_global_route.cpp`

路线执行命令：

```bash
~/semantic_nav_ws/tools/drive_blue_global_route \
  ~/semantic_nav_ws/routes/exp012_global_rectangle.csv \
  ~/semantic_nav_ws/outputs/exp012_global_rectangle_v1
```

实验结果：

| 航点 | Odom 坐标目标 | 最终误差 | 执行时长 | 状态 |
|---|---:|---:|---:|---|
| wp01_north | (0.70, 1.00) | 3.55 mm | 9.00 s | success |
| wp02_west | (0.35, 1.00) | 9.26 mm | 10.46 s | success |
| wp03_south | (0.35, 0.65) | 7.91 mm | 10.45 s | success |
| wp04_east | (0.70, 0.65) | 13.74 mm | 10.47 s | success |

汇总指标：

- 航点成功率：4 / 4；
- 平均最终误差：8.62 mm；
- 最大最终误差：13.74 mm；
- 目标停止阈值：25 mm；
- 总执行时间：40.39 s。

结论：

通过。

本实验实现了 CSV 驱动的固定 odometry 坐标多航点路线执行。
注意：路线坐标使用 `vehicle_blue/odom`，不是 Gazebo 的绝对 world 坐标。

---

### EXP-013：静态障碍物场景下的固定 Odom 坐标绕行路线执行

实验目标：

- 在自定义 Gazebo 世界中加入静态长方体障碍物；
- 基于 `vehicle_blue/odom` 的固定坐标航点执行绕行路线；
- 验证机器人能够沿预定义路线从障碍物上方通过，并到达右侧目标区域；
- 保存路线汇总、误差统计和带障碍物标注的轨迹图；
- 明确区分“预定义绕行路线执行”与“自动避障 / 自动路径规划”。

自定义世界文件：

`~/semantic_nav_ws/worlds/exp013_obstacle_world.sdf`

静态障碍物参数：

- world 坐标中心：`(3.5, 2.0)`；
- 障碍物尺寸：`1.4 m × 1.6 m × 1.0 m`；
- 蓝车初始 world 坐标：`(0.0, 2.0)`；
- 蓝车 odometry 起点：`(0.0, 0.0)`；
- 因此，在 `vehicle_blue/odom` 坐标系中，障碍物中心为：`(3.5, 0.0)`。

路线文件：

`~/semantic_nav_ws/routes/exp013_static_obstacle_detour.csv`

路线控制器：

`~/semantic_nav_ws/tools/drive_blue_global_route.cpp`

路线可视化脚本：

`~/semantic_nav_ws/tools/plot_obstacle_route.py`

固定 odometry 坐标绕行路线：

1. `wp01_move_north`：`(0.0, 4.4)`，先向上远离障碍物；
2. `wp02_pass_above_obstacle`：`(5.5, 4.4)`，从障碍物上方横向通过；
3. `wp03_goal_right`：`(5.5, 2.0)`，沿障碍物右侧下行至目标区域。

控制器调整记录：

- 首次运行中，第二段横向路径较长，在原有单航点 `35 s` 安全超时限制下未能完成；
- 保留安全超时机制，并将单航点超时调整为 `70 s`；
- 调整后重新启动仿真世界并执行完整路线。

实验结果：

| 航点 | Odom 坐标目标 | 最终误差 | 执行时长 | 状态 |
|---|---:|---:|---:|---|
| wp01_move_north | (0.0, 4.4) | 21.46 mm | 33.83 s | success |
| wp02_pass_above_obstacle | (5.5, 4.4) | 23.23 mm | 40.47 s | success |
| wp03_goal_right | (5.5, 2.0) | 20.10 mm | 21.42 s | success |

汇总指标：

- 航点成功率：3 / 3；
- 平均最终误差：21.60 mm；
- 最大最终误差：23.23 mm；
- 目标停止阈值：25 mm；
- 总执行时间：95.73 s；
- 所有航点均进入停止阈值并完成稳定停车。

结论：

通过。

蓝车已能够在包含静态障碍物的自定义 Gazebo 世界中，
读取固定 odometry 坐标系下的预定义航点路线，
先从障碍物上方绕行，再到达其右侧目标区域。

本实验属于“静态障碍物场景下的预定义绕行路线执行”。
当前系统尚未实现障碍物感知、自动路径搜索、在线重规划或自主避障。

---

## 十三、当前能力

- [完成] WSL2 + Ubuntu 22.04.5 开发环境；
- [完成] ROS 2 Humble 基础发布订阅通信；
- [完成] Gazebo Fortress 差速车仿真；
- [完成] Gazebo Transport 直接速度控制；
- [完成] C++ 持久 Publisher 与可靠停车；
- [完成] odometry 的 x、y、quaternion 与 yaw 读取；
- [完成] 闭环距离控制与比例减速；
- [完成] 基于 yaw 的闭环左转和右转；
- [完成] 相对坐标单点导航；
- [完成] 连续相对航点路线执行与轨迹可视化；
- [完成] CSV 驱动的固定 odometry 坐标多航点路线；
- [完成] 自定义静态障碍物 Gazebo world；
- [完成] 静态障碍物场景下的预定义绕行路线；
- [完成] 轨迹 CSV、航点汇总 CSV、误差图与路线图输出。

---

## 十四、当前限制

1. ROS-Gazebo bridge 在当前 WSL 环境中的跨终端发现仍不稳定；
2. 当前稳定控制路径使用 Gazebo Transport，尚未改造成 ROS 2 原生 `rclcpp` 控制节点；
3. 当前路线目标使用 `vehicle_blue/odom` 固定坐标系；仿真重启后 odom 原点会重置，尚未接入全局地图或定位系统；
4. 静态障碍物实验使用预定义绕行航点，尚未实现基于传感器的障碍物检测；
5. 尚未实现 A*、Dijkstra、Nav2 等自动路径搜索与在线重规划；
6. 尚未接入 LiDAR、深度相机、视觉感知或多模态传感器；
7. 尚未实现语言指令解析、场景语义理解或 VLA 决策模块。

---

## 十五、常用复现命令

### 启动基础差速车世界

```bash
source /opt/ros/humble/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch ros_gz_sim_demos diff_drive.launch.py rviz:=false
```

### 启动 EXP-013 静态障碍物世界

```bash
source /opt/ros/humble/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1

ros2 launch ros_gz_sim gz_sim.launch.py \
  gz_args:="-r /home/openclaw/semantic_nav_ws/worlds/exp013_obstacle_world.sdf"
```

### 基础控制

```bash
~/semantic_nav_ws/tools/drive_blue_cpp forward
~/semantic_nav_ws/tools/drive_blue_cpp left
~/semantic_nav_ws/tools/drive_blue_cpp right
~/semantic_nav_ws/tools/drive_blue_cpp route
~/semantic_nav_ws/tools/drive_blue_cpp stop
```

### 高精度距离控制

```bash
~/semantic_nav_ws/tools/drive_blue_distance_slow 0.50
```

### 查看 odometry

```bash
ign topic -e -t /model/vehicle_blue/odometry
```

### 执行 EXP-012 固定 Odom 坐标路线

```bash
~/semantic_nav_ws/tools/drive_blue_global_route \
  ~/semantic_nav_ws/routes/exp012_global_rectangle.csv \
  ~/semantic_nav_ws/outputs/exp012_global_rectangle_v1
```

### 执行 EXP-013 静态障碍物绕行路线

```bash
~/semantic_nav_ws/tools/drive_blue_global_route \
  ~/semantic_nav_ws/routes/exp013_static_obstacle_detour.csv \
  ~/semantic_nav_ws/outputs/exp013_static_obstacle_detour_v1
```

---

## 十六、未来新增实验记录模板

### EXP-XXX：实验名称

目标：

- 本次要验证什么？

运行命令：

```bash
命令写在这里
```

关键参数：

- 参数 1：
- 参数 2：

预期结果：

- 预期机器人会怎样运动？

实际结果：

- 实际发生了什么？

指标：

- 目标值：
- 实际值：
- 绝对误差：
- 相对误差：

问题：

- 遇到了什么异常？

解决方式：

- 改了什么，为什么改？

结论：

- 通过 / 部分通过 / 未通过
