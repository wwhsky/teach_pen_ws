# Teach Pen Workspace

这个工作空间目前主要包含四部分：

- `vive_tracker_ros2`：通过 OpenVR/SteamVR 读取 Vive Tracker 位姿，并发布 ROS2 topic / TF。
- `calibration`：用于标定 `steamvr_base`、`tracker_frame`、`teaching_pen_tip`、`welding_torch_tip`、`workpiece_frame` 等坐标关系。
- `teach_pen`：项目自己的业务包，包含示教路径采集、路径回放、焊缝感知和 JAKA 轨迹执行节点。
- `jaka_ros2`：JAKA 机械臂的描述、MoveIt 配置、仿真和实机接口。

## MoveIt 和执行端的关系

MoveIt 主要负责规划，不直接决定“后面是真机械臂还是仿真机械臂”。

它做的事情大致是：

```text
目标位姿 / 关节目标
-> IK
-> 路径规划
-> 碰撞检查
-> 时间参数化
-> 生成 joint trajectory
-> 发送给轨迹执行接口
```

在 JAKA ZU5 的 MoveIt 配置里，MoveIt 会把轨迹发到这个 action：

```text
/jaka_zu5_controller/follow_joint_trajectory
```

这个 action 不是普通 topic，而是 ROS2 action，类型是：

```text
control_msgs/action/FollowJointTrajectory
```

所以真正决定仿真还是真机的是：**当前是谁提供了 `/jaka_zu5_controller/follow_joint_trajectory` 这个 action server**。

## Controller 配置是什么意思

JAKA ZU5 的 MoveIt 控制器配置在：

```text
src/jaka_ros2/src/jaka_zu5_moveit_config/config/moveit_controllers.yaml
```

核心内容类似：

```yaml
jaka_zu5_controller:
  type: FollowJointTrajectory
  action_ns: follow_joint_trajectory
  joints:
    - joint_1
    - joint_2
    - joint_3
    - joint_4
    - joint_5
    - joint_6
```

它告诉 MoveIt：

```text
控制器名字：jaka_zu5_controller
接口类型：FollowJointTrajectory action
action 名字：follow_joint_trajectory
控制关节：joint_1 ~ joint_6
```

因此 MoveIt 最后会发送到：

```text
/jaka_zu5_controller/follow_joint_trajectory
```

## RViz Fake 仿真

启动：

```bash
cd ~/code/teach_pen_ws
source install/setup.bash
ros2 launch jaka_zu5_moveit_config demo.launch.py use_rviz_sim:=true
```

链路：

```text
MoveIt
-> /jaka_zu5_controller/follow_joint_trajectory
-> fake ros2_control
-> /joint_states
-> robot_state_publisher
-> /tf
-> RViz 里面机械臂运动
```

这个模式没有 Gazebo 物理世界，也没有实机，只是用 fake hardware 模拟关节执行。

## Gazebo 仿真

启动：

```bash
cd ~/code/teach_pen_ws
source install/setup.bash
ros2 launch jaka_zu5_moveit_config demo_gazebo.launch.py
```

链路：

```text
MoveIt
-> /jaka_zu5_controller/follow_joint_trajectory
-> Gazebo ros2_control
-> Gazebo 里的机器人运动
-> /joint_states
-> RViz 同步显示
```

Gazebo 适合测试机器人和工件、环境之间的空间关系，也方便后续加入工件模型。

## teach_pen 节点

当前 `teach_pen` 包里主要有这些节点：

```text
collect_path_node
seam_perception_node
replay_path_node
jaka_trajectory_executor_node
```

### collect_path_node

用于从示教笔采样路径。它从键盘读取控制命令，查询 TF 中的笔尖位姿，并保存为 YAML。

运行示例：

```bash
cd ~/code/teach_pen_ws
source install/setup.bash
ros2 run teach_pen collect_path_node
```

键盘控制：

```text
Space  采样一个点
r      开关连续采样
u      撤销最后一个点
s      保存 YAML
q      退出节点
```

默认输出路径：

```text
config/paths/demo_path.yaml
```

路径 YAML 的基本格式：

```yaml
base_frame: robot_base
tip_frame: welding_torch_tip
sample_count: 3

samples:
  - index: 0
    stamp_sec: 0.0
    translation: [0.35, 0.00, 0.30]
    rotation_xyzw: [0.0, 0.0, 0.0, 1.0]
```

其中 `translation` 单位为米，`rotation_xyzw` 为四元数，顺序是
`x, y, z, w`。

### replay_path_node

用于读取路径 YAML，并通过 MoveIt 回放路径。

当前流程：

```text
读取 samples
-> 检查采样点是否为空
-> 先规划并移动到第一个采样点
-> computeCartesianPath 生成笛卡尔路径
-> IterativeParabolicTimeParameterization 补时间/速度/加速度
-> execute 发送给 MoveIt 当前 controller
```

运行示例：

```bash
cd ~/code/teach_pen_ws
source install/setup.bash
ros2 run teach_pen replay_path_node --ros-args \
  -p model:=zu5 \
  -p input_file:=config/paths/demo_path.yaml
```

注意：`demo_path.yaml` 中的点会被 MoveIt 当作当前 planning frame 下的
TCP 位姿。如果坐标系或姿态不合理，可能出现不可达、IK 失败或碰撞失败。

### seam_perception_node

用于处理 ROI 点云并提取焊缝线。当前实现思路是：

```text
输入 ROI 点云
-> 分割两个平面
-> 计算两平面交线
-> 根据交线附近真实点云支持范围裁剪起点和终点
-> 输出 measured_path
```

当前主要用于验证角焊缝/交线类焊缝的点云提取流程。

### jaka_trajectory_executor_node

这是项目新增的 JAKA 实机轨迹执行节点，用来替代或对照官方
`jaka_planner/moveit_server.cpp`。

它的职责：

```text
连接 JAKA SDK
发布 /joint_states
提供 /jaka_zu5_controller/follow_joint_trajectory
接收 MoveIt 发来的 JointTrajectory
逐点调用 JAKA SDK servo_j 执行
```

运行示例：

```bash
cd ~/code/teach_pen_ws
source install/setup.bash
ros2 run teach_pen jaka_trajectory_executor_node --ros-args \
  -p ip:=<机械臂IP> \
  -p model:=zu5
```

启动后可以检查：

```bash
ros2 action list | grep trajectory
ros2 topic echo /joint_states --once
```

应该能看到：

```text
/jaka_zu5_controller/follow_joint_trajectory
```

这个节点当前是第一版执行器，已经具备基本轨迹接收和 SDK 执行能力。后续
还需要继续加强取消、急停、错误码处理、轨迹时间检查和限速保护。

## 实机执行

实机执行有两条路线。

### 路线 A：使用项目自己的 executor

推荐后续项目主线使用这个方式：

```bash
cd ~/code/teach_pen_ws
source install/setup.bash
ros2 run teach_pen jaka_trajectory_executor_node --ros-args \
  -p ip:=<机械臂IP> \
  -p model:=zu5
```

再启动 MoveIt / RViz：

```bash
cd ~/code/teach_pen_ws
source install/setup.bash
ros2 launch jaka_zu5_moveit_config demo.launch.py
```

链路：

```text
RViz / replay_path_node
-> MoveIt move_group
-> /jaka_zu5_controller/follow_joint_trajectory
-> teach_pen/jaka_trajectory_executor_node
-> JAKA SDK servo_j()
-> 真实机械臂
```

实机模式下不要同时启动：

```bash
ros2 launch jaka_zu5_moveit_config demo.launch.py use_rviz_sim:=true
```

因为 `use_rviz_sim:=true` 会启动 fake ros2_control，可能和实机 executor
抢 `/joint_states` 和 trajectory action。

### 路线 B：使用 JAKA 官方 moveit_server

官方实机链路由 `jaka_planner` 里的 `moveit_server` 接 MoveIt 轨迹，再通过
JAKA SDK 发给机械臂。

启动实机 server：

```bash
cd ~/code/teach_pen_ws
source install/setup.bash
ros2 launch jaka_planner moveit_server.launch.py ip:=<机械臂IP> model:=zu5
```

再启动 MoveIt / RViz：

```bash
cd ~/code/teach_pen_ws
source install/setup.bash
ros2 launch jaka_zu5_moveit_config demo.launch.py
```

`moveit_server.cpp` 中 action server 创建位置：

```cpp
"/jaka_" + robot_model + "_controller/follow_joint_trajectory"
```

如果传入：

```bash
model:=zu5
```

它就会提供：

```text
/jaka_zu5_controller/follow_joint_trajectory
```

这正好和 MoveIt 的控制器配置对上。

官方 `moveit_server.cpp` 更像 demo：取消、停止、错误处理和轨迹时间控制都
比较粗糙。可以作为参考，但最终项目建议逐步切到
`jaka_trajectory_executor_node`。

## 如何确认当前接的是谁

查看 action：

```bash
ros2 action list
```

应该能看到：

```text
/jaka_zu5_controller/follow_joint_trajectory
```

查看节点：

```bash
ros2 node list
```

大致判断：

- 看到 `jaka_trajectory_executor_node`：大概率是项目自己的 JAKA 实机链路。
- 看到 `moveit_server`：大概率是 JAKA 官方实机链路。
- 看到 `controller_manager` / `joint_trajectory_controller`：大概率是 fake ros2_control 或 Gazebo 链路。

## moveit_test 为什么能让 RViz 动

`jaka_planner` 里的：

```bash
ros2 run jaka_planner moveit_test --ros-args -p model:=zu5
```

本质上是一个 MoveIt 客户端。它给 MoveIt 设置目标，然后调用 plan / execute。

如果当前有 fake ros2_control，RViz 里的模型会动。

如果当前有 Gazebo controller，Gazebo 里的模型会动。

如果当前有 `moveit_server` 或 `jaka_trajectory_executor_node` 连着真机，真实机械臂会动。

所以它不是直接控制 RViz，而是通过 MoveIt 走同一套轨迹执行接口。

## 关于 moveit_server.cpp 里的 step_num

`moveit_server.cpp` 收到 MoveIt 轨迹后，会逐点调用 JAKA SDK：

```cpp
robot.servo_j(&joint_pose, MoveMode::ABS, step_num);
```

其中：

```cpp
int step_num = static_cast<int>(dt / 0.008f);
step_num = max(step_num, 1);
```

这里的 `dt` 是相邻两个轨迹点的时间差，`0.008s` 对应 8ms servo 周期。

注意这个实现比较 demo 化：

- `static_cast<int>` 会向下取整，例如 `9ms / 8ms = 1.125` 会变成 `1`。
- 循环里没有按 `dt` 主动 sleep，而是快速把点交给 SDK。
- 轨迹真实执行效果依赖 JAKA SDK / 控制器内部如何处理 `servo_j`。

所以上实机前需要谨慎测试速度、加速度、停止逻辑和误差。

## ZU5 末端工具模型

本次将相机、相机支架和焊枪的组合模型接入了 JAKA ZU5 的 URDF，使
RViz 和 MoveIt 能够显示工具并进行碰撞检查。

### 模型文件

模型位于：

```text
src/jaka_ros2/src/jaka_description/meshes/jaka_zu5_meshes
```

当前实际使用：

```text
Assembly_binary.stl
AssemblySimplified_binary.stl
```

- `Assembly_binary.stl`：较完整的外观模型，用于 `<visual>`。
- `AssemblySimplified_binary.stl`：简化模型，用于 `<collision>`。

原始 CAD 导出的 STL 尺寸单位为毫米，而 URDF 使用米，因此两个 mesh
都设置了：

```xml
scale="0.001 0.001 0.001"
```

RViz 只能可靠读取二进制 STL，所以保留了原始 STL，同时生成了对应的
`*_binary.stl` 文件供 URDF 使用。

### URDF 结构

当前工具结构保持为：

```text
link0
└── robot_base

Link6
└── robot_flange
    └── tool_assembly
```

定义位置：

```text
src/jaka_ros2/src/jaka_description/urdf/jaka_zu5.urdf
```

为了保留 JAKA 原始 link 名称，同时让项目代码使用含义更清楚的坐标系，
URDF 中增加了两个零偏置语义 frame：

```text
link0 -> robot_base
Link6 -> robot_flange
```

它们分别与 JAKA 的 `link0` 和 `Link6` 完全重合。`tool_assembly` 通过
固定关节连接到 `robot_flange`：

```xml
<joint name="robot_flange_to_tool_assembly" type="fixed">
  <origin xyz="0 0 0" rpy="0 0 0" />
  <parent link="robot_flange" />
  <child link="tool_assembly" />
</joint>
```

当前组合 STL 已经包含相机、支架和焊枪，所以没有再为这些实体分别建立
可视化 link。`camera_frame` 和焊枪 TCP 属于功能坐标系，后续可以根据
手眼标定和工具标定结果单独通过 TF 发布，不要求拆分组合 STL。

`Link6` 本身不是由 URDF 节点主动测量出来的。运行时：

```text
/joint_states
-> robot_state_publisher
-> 根据 URDF 正运动学计算
-> 发布 Link1 ... Link6 以及固定工具关节的 TF
```

可以使用下面的命令检查：

```bash
ros2 topic echo /joint_states
ros2 run tf2_ros tf2_echo Link5 Link6
ros2 run tf2_ros tf2_echo link0 robot_base
ros2 run tf2_ros tf2_echo Link6 robot_flange
ros2 run tf2_ros tf2_echo robot_flange tool_assembly
```

### MoveIt 碰撞配置

工具的碰撞模型会参与 MoveIt/FCL 的碰撞检查。URDF 中存在
`<collision>` 只表示“这是碰撞几何”，并不表示当前已经发生碰撞。

`robot_flange` 是没有碰撞几何的语义 frame，工具在物理上仍然安装于
`Link6` 法兰处。为了允许安装面附近的合理几何重叠，`Link6` 与
`tool_assembly` 之间的碰撞继续在 SRDF 中忽略：

```xml
<disable_collisions link1="Link6" link2="tool_assembly" reason="Adjacent"/>
```

配置位置：

```text
src/jaka_ros2/src/jaka_zu5_moveit_config/config/jaka_zu5.srdf
```

如果日志提示工具与 `Link3`、`Link4` 等非相邻连杆碰撞，不应直接加入
忽略列表。应优先检查：

1. `robot_flange_to_tool_assembly` 的 `xyz` 和 `rpy` 是否符合 CAD 装配坐标。
2. STL 原点和坐标轴是否与 `Link6` 坐标系一致。
3. STL 是否正确使用毫米到米的缩放。
4. 简化碰撞模型是否包含了过大的包络或无关几何。
5. 当前机械臂起始姿态是否确实使工具碰到了本体。

必要时可以暂时注释工具的 `<collision>`，只用于判断规划失败是否由工具
碰撞模型引起；确认原因后应恢复碰撞检查。

### 修改后的构建流程

修改 URDF、mesh 或 SRDF 后，需要重新构建对应包并重新启动 MoveIt：

```bash
cd ~/code/teach_pen_ws
colcon build --packages-select jaka_description jaka_zu5_moveit_config
source install/setup.bash
ros2 launch jaka_zu5_moveit_config demo.launch.py use_rviz_sim:=true
```

仅在 RViz 中重新加载显示，通常不会使已经运行的
`robot_state_publisher` 和 `move_group` 自动读取新模型，最稳妥的做法是
结束原 launch 后重新启动。

### 当前进展与下一步

已经完成：

- 组合工具 STL 接入 ZU5 URDF。
- 外观模型和简化碰撞模型分开使用。
- 完成毫米到米的模型缩放。
- 添加 `link0 -> robot_base` 和 `Link6 -> robot_flange` 语义坐标系。
- 添加 `robot_flange -> tool_assembly` 固定关节。
- 在 SRDF 中忽略相邻的 `Link6` 与 `tool_assembly` 碰撞。
- 在 RViz/MoveIt 中完成基本显示和规划验证。

下一步：

- 从 CAD 确认组合 STL 相对于 `robot_flange` 的准确安装变换。
- 完成焊枪 TCP 标定，得到工具坐标系相对 `robot_flange` 的位姿。
- 完成相机手眼标定，发布相机坐标系 TF。
- 用多组机械臂姿态验证工具不会与本体发生错误碰撞。
- 将焊缝路径转换为焊枪 TCP 路径，并在 MoveIt 中进行笛卡尔规划。
