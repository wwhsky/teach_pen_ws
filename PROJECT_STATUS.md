# 无线示教笔项目进展整理

## 1. 项目目标

本项目目标是实现一个无线示教笔系统：

用户使用带有 Vive Tracker 的示教笔采集空间路径，系统将示教笔路径转换到机器人坐标系下，再结合工具标定、工件标定、焊缝点云感知和 MoveIt 规划，最终驱动 JAKA ZU5 机械臂执行焊接轨迹。

整体数据链路可以理解为：

```text
Vive Tracker 位姿
  -> teaching_pen_tip
  -> robot_base 坐标系下的示教路径
  -> 工具 / 工件 / VR 到机器人标定
  -> 焊缝点云修正
  -> welding_torch_tip 轨迹
  -> MoveIt 规划
  -> JAKA 执行
```

## 2. Vive Tracker 模块

包路径：

```text
src/vive_tracker_ros2
```

作用：

- 通过 OpenVR 读取 Vive Tracker 3.0 的位姿。
- 将 Tracker 位姿发布为 ROS2 topic 和 TF。
- 后续作为示教笔空间定位来源。

当前进展：

- OpenVR 节点已经可以启动。
- SteamVR 已配置为无 HMD 模式，当前不强依赖头显。
- Tracker 可以被 SteamVR / OpenVR 识别。
- 当前检测到的设备示例：

```text
serial=LHR-D288AEEE
model=VIVE Tracker 3.0 MV
class=GenericTracker
```

- 没有基站时，设备可以连接，但位姿无效：

```text
pose_valid=false
tracking_result=Calibrating_OutOfRange
```

- 已实现的发布内容：
  - Tracker pose topic
  - `steamvr_base -> tracker_frame` TF
  - 设备 debug 信息
  - 初步按钮 topic

按钮相关：

- 当前尝试读取 Tracker 按钮状态。
- 已设计 topic：

```text
/vive_tracker/buttons
/vive_tracker/<serial>/buttons
```

- 但当前 OpenVR `GetControllerState()` 对该 Tracker 返回不可用：

```text
buttons=<unavailable>
```

待完成：

- 等基站到货后验证真实 6D 位姿。
- 确认 Tracker 坐标是否稳定。
- 继续排查按钮输入。
- 如果 Tracker 外设输入不可用，考虑用外部 MCU / 串口 / 蓝牙按钮作为备选方案。

## 3. 标定模块

包路径：

```text
src/calibration
```

作用：

- 完成各坐标系之间的标定。
- 将标定结果保存为 YAML。
- 通过 ROS2 发布静态 TF。

当前坐标树设计：

```text
robot_base
├── robot_flange
│   ├── camera_frame
│   └── welding_torch_tip
├── steamvr_base
│   └── tracker_frame
│       └── teaching_pen_tip
└── workpiece_frame
```

当前已有程序：

```text
calibrate_vr_to_robot.py
calibrate_tool.py
calibrate_workpiece.py
publish_calibration_tf.py
```

各程序作用：

- `calibrate_vr_to_robot.py`
  - 标定 `robot_base -> steamvr_base`。
  - 使用 `robot_base` 下的焊枪尖点位和 `steamvr_base` 下的示教笔尖点位配准。

- `calibrate_tool.py`
  - 用于工具尖端标定。
  - 可用于：

```text
tracker_frame -> teaching_pen_tip
robot_flange -> welding_torch_tip
```

- `calibrate_workpiece.py`
  - 三点法标定工件坐标系。
  - 第一点：工件原点。
  - 第二点：确定 X 轴。
  - 第三点：确定 XY 平面。

- `publish_calibration_tf.py`
  - 读取 YAML。
  - 发布静态 TF。

配置文件目录：

```text
config/calibration
```

当前已有示例配置：

```text
vr_to_robot.yaml
teaching_pen_tip.yaml
welding_torch_tip.yaml
workpiece.yaml
```

待完成：

- 用真实硬件采点验证标定流程。
- 明确 `welding_torch_tip` 的 TCP 定义。
- 后续补充工具姿态标定，目前可以先手动修改 YAML 中的旋转部分。

## 4. JAKA / MoveIt 模块

包路径：

```text
src/jaka_ros2
```

作用：

- 提供 JAKA 机器人模型。
- 提供 MoveIt 配置。
- 支持 RViz fake 仿真、Gazebo 仿真和实机控制。

当前理解：

MoveIt 主要负责规划轨迹，不直接决定轨迹由谁执行。轨迹会发送到 controller action，具体由哪个 controller 接收，决定了是仿真还是实机。

几种模式：

```text
MoveIt
  -> fake controller
      -> RViz 中的虚拟机器人运动

MoveIt
  -> Gazebo ros2_control
      -> Gazebo 仿真机器人运动

MoveIt
  -> jaka_planner/moveit_server
      -> JAKA SDK
          -> 真机运动
```

当前进展：

- `demo.launch.py` 可以启动 MoveIt / RViz。
- `moveit_test.cpp` 可以让 RViz 中的机械臂运动。
- 已阅读并分析：

```text
src/jaka_ros2/src/jaka_planner/src/moveit_test.cpp
src/jaka_ros2/src/jaka_planner/src/moveit_server.cpp
```

- 当前判断：官方 `moveit_server.cpp` 更像 demo，不一定适合作为最终实机执行节点。

发现的问题：

- 轨迹执行逻辑较粗糙。
- cancel / stop / error handling 不完整。
- `step_num = int(dt / 0.008)` 存在时间截断问题。
- 对实机执行安全性和稳定性还需要进一步验证。

待完成：

- 先用官方 server 跑通实机。
- 如果官方 server 不稳定，再编写自己的 trajectory server。
- 最终需要将示教路径 / 焊缝路径接入 MoveIt 规划流程。

## 5. teach_pen 主控模块

包路径：

```text
src/teach_pen
```

作用：

- 项目自己的主业务包。
- 负责示教路径采集、焊缝感知、路径生成、后续流程控制。

当前已有节点：

```text
collect_path_node.cpp
seam_perception_node.cpp
```

### 5.1 collect_path_node

作用：

- 订阅示教笔按钮。
- 查询 TF 中的示教笔尖位置。
- 采集示教点。
- 保存示教路径。

默认查询：

```text
robot_base <- teaching_pen_tip
```

当前按钮设计：

```text
trigger  -> 采一个点
grip     -> 连续记录开关
trackpad -> 撤销
menu     -> 保存
```

当前进展：

- 节点已初步完成。
- 可以根据按钮 topic 执行采点逻辑。
- 当前受限于 Tracker 按钮暂时无法读取。

待完成：

- 接入真实按钮输入。
- 稳定路径保存格式。
- 将采集路径转换为机器人可执行路径。

### 5.2 seam_perception_node

作用：

- 从 ROI 点云中提取焊缝线。
- 当前主要针对两平面相交形成的角焊缝。

输入：

```text
/seam_camera/roi_points
```

也支持从离线 `.ply` 文件读取点云测试。

输出：

```text
/seam_tracking/measured_path
/seam_tracking/debug_cloud
```

当前算法：

```text
点云
  -> 去除 NaN
  -> voxel 降采样
  -> RANSAC 拟合第一个平面
  -> RANSAC 拟合第二个非平行平面
  -> 求两个平面的交线
  -> 找交线附近同时被两个平面点云支持的区间
  -> 输出焊缝路径
```

当前进展：

- 已经可以读取 `corner_point_right.ply` 测试点云。
- 已经能提取两平面交线。
- 已修正路径过长的问题：
  - 之前直接把所有平面点投影到交线上，路径会超过真实焊接区域。
  - 当前改为只选择交线附近确实有点云支撑的部分。
- RViz 中已经能看到较合理的焊缝 path 和 debug cloud。

待完成：

- 将当前两点 path 改成按间距采样的多点 path。
- 输出或保存平面法向量、焊缝方向、角平分方向。
- 从几何焊缝线生成真正的焊枪 TCP 轨迹。
- 加入焊接工艺参数：

```text
焊枪偏移
干伸长
工作角
行走角
是否贴线 / 偏线 / 抬高
```

## 6. 焊缝线与焊枪轨迹的关系

当前点云算法提取出来的是几何焊缝线，也就是两个工件平面的交线。

但实际焊接时，机器人执行的是：

```text
welding_torch_tip 的轨迹
```

这个轨迹不一定严格等于几何交线。

它取决于：

- `welding_torch_tip` 的 TCP 定义。
- 焊枪干伸长。
- 焊枪工作角。
- 焊枪行走角。
- 焊接时是否需要偏离交线。
- 工艺要求中电弧中心、焊丝尖端和焊缝根部之间的关系。

后续应该形成这样的路径生成链路：

```text
measured_seam_line
  -> 加工艺偏移和姿态
  -> welding_torch_tip path
  -> MoveIt plan
  -> JAKA execute
```

## 7. 当前下一步工作建议

优先级较高的工作：

1. 完善 `seam_perception_node`
   - 将两点 path 改为多点采样 path。
   - 增加焊缝方向和法向信息。
   - 为后续生成焊枪姿态做准备。

2. 明确焊枪 TCP 定义
   - `welding_torch_tip` 是焊丝尖端、电弧中心，还是焊枪喷嘴上的某个点。
   - 这个定义会影响后续所有路径生成。

3. 编写路径执行节点
   - 先只接 MoveIt / RViz。
   - 不急着接实机。
   - 流程为：

```text
焊缝路径 / 示教路径
  -> 生成 welding_torch_tip poses
  -> MoveIt 规划
  -> RViz 中验证机械臂运动
```

4. 等硬件到齐后验证
   - Vive 基站。
   - Tracker 位姿。
   - 按钮输入。
   - 示教路径采集。
   - 标定流程。
   - JAKA 实机执行。

## 8. 当前风险点

- Tracker 按钮输入目前不可用，需要后续确认。
- 没有基站时无法验证真实 Tracker 位姿。
- JAKA 官方 `moveit_server.cpp` 不一定满足最终实机执行需求。
- 焊缝点云提取目前只是初版，后续需要适配更多焊缝类型和真实相机噪声。
- 几何焊缝线不等于实际焊枪轨迹，需要加入焊接工艺模型。

