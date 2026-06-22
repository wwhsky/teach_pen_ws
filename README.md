# Teach Pen Workspace

这个工作空间目前主要包含三部分：

- `vive_tracker_ros2`：通过 OpenVR/SteamVR 读取 Vive Tracker 位姿，并发布 ROS2 topic / TF。
- `calibration`：用于标定 `steamvr_base`、`tracker_frame`、`teaching_pen_tip`、`welding_torch_tip`、`workpiece_frame` 等坐标关系。
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

## 实机执行

实机链路由 `jaka_planner` 里的 `moveit_server` 接 MoveIt 轨迹，再通过 JAKA SDK 发给机械臂。

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

链路：

```text
RViz / moveit_test
-> MoveIt move_group
-> /jaka_zu5_controller/follow_joint_trajectory
-> jaka_planner/moveit_server.cpp
-> JAKA SDK servo_j()
-> 真实机械臂
```

`moveit_server.cpp` 中对应的 action server 创建位置：

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

- 看到 `moveit_server`：大概率是 JAKA 实机链路。
- 看到 `controller_manager` / `joint_trajectory_controller`：大概率是 fake ros2_control 或 Gazebo 链路。

## moveit_test 为什么能让 RViz 动

`jaka_planner` 里的：

```bash
ros2 run jaka_planner moveit_test --ros-args -p model:=zu5
```

本质上是一个 MoveIt 客户端。它给 MoveIt 设置目标，然后调用 plan / execute。

如果当前有 fake ros2_control，RViz 里的模型会动。

如果当前有 Gazebo controller，Gazebo 里的模型会动。

如果当前有 `moveit_server` 连着真机，真实机械臂会动。

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
